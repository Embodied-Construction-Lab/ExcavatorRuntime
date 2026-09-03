"""Read-only capture of complete TADPS selector candidate frames.

The selector publishes evidence over ROS.  This module owns only local evidence
files: an exclusive append-only JSONL trace and an atomically published manifest.
It has no motion-control dependency.
"""

from __future__ import annotations

import hashlib
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from mission.tadps_replay_export import (
    TadpsReplayExportError,
    _parse_frame_record,
    _unique_object,
)


class TadpsLiveCaptureError(ValueError):
    """A live frame or its source provenance is unsafe to record as evidence."""


class TadpsLiveCaptureSession:
    """Capture one selector sequence into a new, exclusive evidence directory."""

    def __init__(
        self,
        *,
        output_directory: Path,
        expected_sequence_id: str,
        source_topic: str,
        selector_source_snapshot: Mapping[str, Any],
        maximum_payload_bytes: int = 64 * 1024 * 1024,
    ) -> None:
        self.output_directory = output_directory.expanduser()
        self.expected_sequence_id = _nonempty_string(
            expected_sequence_id, "expected_sequence_id"
        )
        self.source_topic = _nonempty_string(source_topic, "source_topic")
        if not isinstance(maximum_payload_bytes, int) or isinstance(
            maximum_payload_bytes, bool
        ) or not 1024 <= maximum_payload_bytes <= 256 * 1024 * 1024:
            raise TadpsLiveCaptureError("maximum_payload_bytes is invalid")
        self.maximum_payload_bytes = maximum_payload_bytes
        self.selector_source_snapshot = _json_clone(selector_source_snapshot)
        self._validate_snapshot(self.selector_source_snapshot)

        self.output_directory.mkdir(mode=0o750, parents=False, exist_ok=False)
        self.trace_path = self.output_directory / "candidate_frames.jsonl"
        self.manifest_path = self.output_directory / "capture_manifest.json"
        self._trace_fd = os.open(
            self.trace_path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_APPEND,
            0o640,
        )
        self._started_at = _utc_now()
        self._frame_count = 0
        self._first_frame_index: int | None = None
        self._last_frame_index: int | None = None
        self._last_stamp_s: float | None = None
        self._frame_id: str | None = None
        self._closed = False
        self._finalized = False

    def record_json(self, payload: str) -> None:
        """Validate then append exactly one complete candidate frame."""

        if self._closed:
            raise TadpsLiveCaptureError("capture session is closed")
        if not isinstance(payload, str):
            raise TadpsLiveCaptureError("candidate frame payload must be text")
        encoded = payload.encode("utf-8")
        if len(encoded) > self.maximum_payload_bytes:
            raise TadpsLiveCaptureError("candidate frame payload exceeds size limit")
        try:
            value = json.loads(payload, object_pairs_hook=_unique_object)
            parsed = _parse_frame_record(value)
        except (json.JSONDecodeError, TadpsReplayExportError) as exc:
            raise TadpsLiveCaptureError(f"invalid candidate frame: {exc}") from exc
        if parsed["sequence_id"] != self.expected_sequence_id:
            raise TadpsLiveCaptureError("candidate frame sequence_id does not match run")
        if self._first_frame_index is None and parsed["frame_index"] != 0:
            raise TadpsLiveCaptureError("candidate frame sequence must start at zero")
        if self._frame_id is not None and parsed["frame_id"] != self._frame_id:
            raise TadpsLiveCaptureError("candidate frame_id changed during capture")
        if (
            self._last_frame_index is not None
            and parsed["frame_index"] != self._last_frame_index + 1
        ):
            raise TadpsLiveCaptureError(
                "candidate frame_index must be contiguous; evidence may have dropped"
            )
        if (
            self._last_stamp_s is not None
            and parsed["stamp_s"] <= self._last_stamp_s
        ):
            raise TadpsLiveCaptureError(
                "candidate frame stamp_s must be strictly increasing"
            )

        canonical = (
            json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
            + "\n"
        ).encode("utf-8")
        _write_all(self._trace_fd, canonical)
        if self._first_frame_index is None:
            self._first_frame_index = parsed["frame_index"]
            self._frame_id = parsed["frame_id"]
        self._last_frame_index = parsed["frame_index"]
        self._last_stamp_s = parsed["stamp_s"]
        self._frame_count += 1

    def close(self) -> Path:
        """Durably close the trace and atomically publish its manifest."""

        if self._finalized:
            return self.manifest_path
        if self._closed:
            raise TadpsLiveCaptureError("capture session closed without a manifest")
        if self._frame_count == 0:
            self.abort()
            raise TadpsLiveCaptureError("capture contains no candidate frames")
        os.fsync(self._trace_fd)
        os.close(self._trace_fd)
        self._closed = True
        trace_sha256 = _file_sha256(self.trace_path)
        manifest = {
            "schema_version": "tadps_live_capture_manifest.v1",
            "record_type": "complete_preselection_candidate_capture",
            "source_topic": self.source_topic,
            "sequence_id": self.expected_sequence_id,
            "frame_id": self._frame_id,
            "frame_count": self._frame_count,
            "first_frame_index": self._first_frame_index,
            "last_frame_index": self._last_frame_index,
            "started_at_utc": self._started_at,
            "completed_at_utc": _utc_now(),
            "candidate_frames_file": self.trace_path.name,
            "candidate_frames_sha256": trace_sha256,
            "selector_source_snapshot": self.selector_source_snapshot,
        }
        _atomic_write_json(self.manifest_path, manifest)
        self._finalized = True
        return self.manifest_path

    def abort(self) -> None:
        """Close an incomplete trace without publishing a misleading manifest."""

        if not self._closed:
            os.close(self._trace_fd)
            self._closed = True

    def __enter__(self) -> "TadpsLiveCaptureSession":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        if exc_type is None:
            self.close()
        else:
            self.abort()

    @staticmethod
    def _validate_snapshot(value: Any) -> None:
        if not isinstance(value, Mapping):
            raise TadpsLiveCaptureError("selector_source_snapshot must be an object")
        required = {
            "schema_version",
            "bridge_id",
            "descriptor_sha256",
            "patch_sha256",
            "source_files",
            "config_files",
        }
        if (
            set(value) != required
            or value.get("schema_version")
            != "tadps_selector_source_snapshot.v1"
        ):
            raise TadpsLiveCaptureError("selector_source_snapshot fields are invalid")
        _nonempty_string(value.get("bridge_id"), "bridge_id")
        for field in ("descriptor_sha256", "patch_sha256"):
            _sha256(value.get(field), field)
        for group in ("source_files", "config_files"):
            files = value.get(group)
            if not isinstance(files, Mapping) or not files:
                raise TadpsLiveCaptureError(f"{group} must be a non-empty object")
            for relative_path, digest in files.items():
                _safe_relative_path(relative_path, group)
                _sha256(digest, f"{group}.{relative_path}")


def load_selector_bridge_descriptor(descriptor_path: Path) -> dict[str, Any]:
    """Strictly load a self-contained bridge descriptor and its versioned patch."""

    unresolved = descriptor_path.expanduser()
    if unresolved.is_symlink():
        raise TadpsLiveCaptureError("bridge provenance descriptor must not be a symlink")
    descriptor = unresolved.resolve(strict=True)
    if not descriptor.is_file():
        raise TadpsLiveCaptureError("bridge provenance descriptor must be a regular file")
    try:
        value = json.loads(
            descriptor.read_text(encoding="utf-8"), object_pairs_hook=_unique_object
        )
    except (UnicodeDecodeError, json.JSONDecodeError, TadpsReplayExportError) as exc:
        raise TadpsLiveCaptureError("bridge provenance descriptor is invalid JSON") from exc
    fields = {
        "schema_version",
        "bridge_id",
        "package_name",
        "patch_file",
        "patch_sha256",
        "base_files",
        "patched_source_files",
        "patched_config_files",
        "runtime_contract",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise TadpsLiveCaptureError("bridge provenance fields are invalid")
    if value["schema_version"] != "tadps_selector_bridge_provenance.v1":
        raise TadpsLiveCaptureError("unsupported bridge provenance schema")
    _nonempty_string(value["bridge_id"], "bridge_id")
    if value["package_name"] != "excavator_dig_point":
        raise TadpsLiveCaptureError("bridge package_name is invalid")
    patch_relative = _safe_relative_path(value["patch_file"], "patch_file")
    patch = _resolve_regular_scoped_file(descriptor.parent, patch_relative)
    expected_patch_sha = _sha256(value["patch_sha256"], "patch_sha256")
    if _file_sha256(patch) != expected_patch_sha:
        raise TadpsLiveCaptureError("bridge patch SHA-256 does not match descriptor")

    _validate_digest_map(value["base_files"], "base_files")
    _validate_digest_map(value["patched_source_files"], "patched_source_files")
    _validate_digest_map(value["patched_config_files"], "patched_config_files")
    _validate_runtime_contract(value["runtime_contract"])
    return _json_clone(value)


def verify_selector_bridge_provenance(
    *, descriptor_path: Path, selector_source_root: Path
) -> dict[str, Any]:
    """Verify the versioned bridge and the exact selector source used by a run."""

    unresolved = descriptor_path.expanduser()
    value = load_selector_bridge_descriptor(unresolved)
    descriptor = unresolved.resolve(strict=True)
    expected_patch_sha = value["patch_sha256"]
    bridge_id = value["bridge_id"]

    unresolved_root = selector_source_root.expanduser()
    if unresolved_root.is_symlink():
        raise TadpsLiveCaptureError("selector_source_root must be a real directory")
    root = unresolved_root.resolve(strict=True)
    if not root.is_dir():
        raise TadpsLiveCaptureError("selector_source_root must be a real directory")
    source_files = _verify_digest_map(
        root, value["patched_source_files"], "patched_source_files"
    )
    config_files = _verify_digest_map(
        root, value["patched_config_files"], "patched_config_files"
    )
    if set(source_files).intersection(config_files):
        raise TadpsLiveCaptureError("source and config provenance files overlap")
    return {
        "schema_version": "tadps_selector_source_snapshot.v1",
        "bridge_id": bridge_id,
        "descriptor_sha256": _file_sha256(descriptor),
        "patch_sha256": expected_patch_sha,
        "source_files": source_files,
        "config_files": config_files,
    }


def _json_clone(value: Any) -> Any:
    try:
        return json.loads(json.dumps(value, ensure_ascii=False, sort_keys=True))
    except (TypeError, ValueError) as exc:
        raise TadpsLiveCaptureError("selector_source_snapshot is not JSON") from exc


def _nonempty_string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TadpsLiveCaptureError(f"{field} must be a non-empty string")
    return value


def _sha256(value: Any, field: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise TadpsLiveCaptureError(f"{field} must be a lowercase SHA-256 digest")
    return value


def _safe_relative_path(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise TadpsLiveCaptureError(f"{field} path must be non-empty")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or "." in path.parts:
        raise TadpsLiveCaptureError(f"{field} path must stay relative")
    return value


def _validate_digest_map(value: Any, field: str) -> dict[str, str]:
    if not isinstance(value, Mapping) or not value:
        raise TadpsLiveCaptureError(f"{field} must be a non-empty object")
    result: dict[str, str] = {}
    for relative_path, digest in value.items():
        path = _safe_relative_path(relative_path, field)
        result[path] = _sha256(digest, f"{field}.{path}")
    return result


def _verify_digest_map(root: Path, value: Any, field: str) -> dict[str, str]:
    expected = _validate_digest_map(value, field)
    for relative_path, digest in expected.items():
        source = _resolve_regular_scoped_file(root, relative_path)
        if _file_sha256(source) != digest:
            raise TadpsLiveCaptureError(
                f"{field}.{relative_path} SHA-256 does not match descriptor"
            )
    return expected


def _resolve_regular_scoped_file(root: Path, relative_path: str) -> Path:
    candidate = root / relative_path
    if candidate.is_symlink():
        raise TadpsLiveCaptureError(
            f"provenance file must not be a symlink: {relative_path}"
        )
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root.resolve(strict=True))
    except (FileNotFoundError, ValueError) as exc:
        raise TadpsLiveCaptureError(
            f"provenance file escapes its declared root: {relative_path}"
        ) from exc
    if not resolved.is_file():
        raise TadpsLiveCaptureError(f"provenance path is not a file: {relative_path}")
    return resolved


def _validate_runtime_contract(value: Any) -> None:
    expected = {
        "enabled_parameter": "candidate_trace_enabled",
        "default_enabled": False,
        "topic_parameter": "candidate_trace_topic",
        "default_topic": "/digging/tadps_candidate_frame",
        "sequence_id_parameter": "candidate_trace_sequence_id",
        "message_type": "std_msgs/msg/String",
        "payload_schema": "tadps_selector_candidate_frame.v1",
        "record_type": "complete_preselection_candidate_set",
        "motion_commands_emitted": 0,
    }
    if value != expected:
        raise TadpsLiveCaptureError("bridge runtime_contract is invalid")


def _write_all(file_descriptor: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        written = os.write(file_descriptor, view)
        if written <= 0:
            raise OSError("append-only trace write made no progress")
        view = view[written:]


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    encoded = (
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    descriptor = os.open(
        temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o640
    )
    try:
        _write_all(descriptor, encoded)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    os.replace(temporary, path)
    directory = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
