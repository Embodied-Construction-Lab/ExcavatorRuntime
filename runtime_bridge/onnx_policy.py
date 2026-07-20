"""ONNX Runtime policy wrapper：38维 observation -> 4维归一化动作。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np


class OnnxPolicyLoadError(RuntimeError):
    """ONNX policy 无法加载或运行。"""


class OnnxPolicy:
    """最小 ONNX Runtime 推理封装，适配 ML-Agents 导出的连续动作模型。"""

    OBSERVATION_INPUT = "obs_0"
    DETERMINISTIC_ACTION_OUTPUT = "deterministic_continuous_actions"

    def __init__(self, model_path: Path, providers: Sequence[str] | None = None) -> None:
        try:
            import onnxruntime as ort
        except ModuleNotFoundError as exc:
            raise OnnxPolicyLoadError(
                "未安装 onnxruntime。请在当前 Python 环境安装后再启动 policy bridge，例如："
                " python3 -m pip install onnxruntime"
            ) from exc

        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise OnnxPolicyLoadError(f"ONNX模型不存在: {self.model_path}")

        selected_providers = list(providers) if providers else ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(str(self.model_path), providers=selected_providers)
        self.input_infos = list(self.session.get_inputs())
        self.output_infos = list(self.session.get_outputs())
        self.observation_input = self._find_observation_input()
        self.action_output = self._find_action_output()

    def run(self, observation: Sequence[float]) -> list[float]:
        """执行一次推理，原样返回有限的 [boom, stick, bucket, swing]。"""
        obs = np.asarray(list(observation), dtype=np.float32)
        if obs.shape != (38,):
            raise ValueError(f"ONNX observation 必须是38维，实际为 {obs.shape}")
        if not np.all(np.isfinite(obs)):
            raise ValueError("ONNX observation 包含非有限值")

        feed: dict[str, Any] = {}
        for input_info in self.input_infos:
            if input_info.name == self.observation_input.name:
                feed[input_info.name] = self._reshape_observation(obs, input_info.shape)
            else:
                feed[input_info.name] = self._zero_input(input_info)

        outputs = self.session.run([self.action_output.name], feed)
        return self._extract_action(outputs)

    def _find_observation_input(self) -> Any:
        """选择并验证部署契约要求的38维 float observation 输入。"""
        for info in self.input_infos:
            if info.name != self.OBSERVATION_INPUT:
                continue
            shape = list(info.shape or [])
            if str(info.type) == "tensor(float)" and len(shape) == 2 and shape[-1] == 38:
                return info
            raise OnnxPolicyLoadError(
                f"ONNX输入 {self.OBSERVATION_INPUT} 签名不匹配: "
                f"type={info.type}, shape={shape}; 期望 tensor(float) [batch, 38]"
            )
        raise OnnxPolicyLoadError(f"ONNX模型缺少输入: {self.OBSERVATION_INPUT}")

    def _find_action_output(self) -> Any:
        """选择并验证部署使用的4维确定性连续动作输出。"""
        for info in self.output_infos:
            if info.name != self.DETERMINISTIC_ACTION_OUTPUT:
                continue
            shape = list(info.shape or [])
            if str(info.type) == "tensor(float)" and len(shape) == 2 and shape[-1] == 4:
                return info
            raise OnnxPolicyLoadError(
                f"ONNX输出 {self.DETERMINISTIC_ACTION_OUTPUT} 签名不匹配: "
                f"type={info.type}, shape={shape}; 期望 tensor(float) [batch, 4]"
            )
        raise OnnxPolicyLoadError(
            f"ONNX模型缺少确定性动作输出: {self.DETERMINISTIC_ACTION_OUTPUT}"
        )

    @staticmethod
    def _reshape_observation(obs: np.ndarray, shape: Sequence[Any]) -> np.ndarray:
        """按模型输入 rank reshape observation；动态 batch 维用 1。"""
        if not shape or len(shape) == 1:
            return obs
        concrete = [1 if not isinstance(dim, int) or dim <= 0 else int(dim) for dim in shape]
        if 38 in concrete:
            concrete[concrete.index(38)] = 38
        if int(np.prod(concrete)) == 38:
            return obs.reshape(concrete)
        if len(concrete) == 2:
            return obs.reshape(1, 38)
        raise OnnxPolicyLoadError(f"无法把38维observation reshape到模型输入shape={shape}")

    @staticmethod
    def _zero_input(input_info: Any) -> np.ndarray:
        """为非 observation 输入构造零值，例如可选 recurrent/action mask 输入。"""
        dtype = np.float32
        type_text = str(input_info.type)
        if "int64" in type_text:
            dtype = np.int64
        elif "int32" in type_text:
            dtype = np.int32
        elif "bool" in type_text:
            dtype = np.bool_

        shape = [1 if not isinstance(dim, int) or dim <= 0 else int(dim) for dim in input_info.shape]
        if not shape:
            shape = [1]
        return np.zeros(shape, dtype=dtype)

    @staticmethod
    def _extract_action(outputs: Sequence[Any]) -> list[float]:
        """从 ONNX outputs 中找到4维有限连续动作，不做数值变换。"""
        arrays = [np.asarray(output) for output in outputs]
        candidates = [array for array in arrays if array.size == 4]
        if not candidates:
            candidates = [array for array in arrays if array.size >= 4 and np.issubdtype(array.dtype, np.number)]
        if not candidates:
            raise OnnxPolicyLoadError("ONNX输出中找不到4维动作")

        action = candidates[0].astype(np.float32).reshape(-1)[:4]
        if not np.all(np.isfinite(action)):
            raise OnnxPolicyLoadError("ONNX动作输出包含非有限值")
        return [float(value) for value in action]
