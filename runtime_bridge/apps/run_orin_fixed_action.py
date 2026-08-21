#!/usr/bin/python3
"""Run one named Orin fixed action through its blocking behavior RPC."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable, Sequence

from runtime_bridge.orin_behavior_rpc import (
    OrinBehaviorConnectionError,
    OrinBehaviorProtocolError,
)
from runtime_bridge.orin_follow_client import (
    FixedActionRejected,
    OrinFollowClient,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("behavior", choices=("ExecuteDig", "ExecuteDump"))
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=18083)
    return parser


def run(
    argv: Sequence[str] | None = None,
    *,
    client_factory: Callable[[str, int], OrinFollowClient] = OrinFollowClient,
) -> int:
    args = _parser().parse_args(argv)
    try:
        client = client_factory(args.host, args.port)
        result = client.run_fixed_action(
            args.behavior,
            feedback_callback=lambda update: print(
                "fixed-action feedback: "
                f"step={update.step_index} label={update.step_label} "
                f"phase={update.phase} max_error={update.max_error:.6f} "
                f"datagrams={update.action_datagrams}",
                flush=True,
            ),
        )
        if (
            result.outcome != "SUCCEEDED"
            or result.reason_code != "SEQUENCE_COMPLETED"
            or not result.quiescence_confirmed
        ):
            raise RuntimeError(
                "fixed action did not finish with confirmed quiescence: "
                f"outcome={result.outcome} reason={result.reason_code} "
                f"quiescence={result.quiescence_confirmed}"
            )
        print(
            f"{result.behavior} complete: {result.reason_code} "
            f"quiescence_confirmed=True datagrams={result.action_datagrams}",
            flush=True,
        )
        return 0
    except (
        FixedActionRejected,
        OrinBehaviorConnectionError,
        OrinBehaviorProtocolError,
        RuntimeError,
        ValueError,
    ) as exc:
        print(f"fixed action failed: {exc}", file=sys.stderr, flush=True)
        return 2
    except KeyboardInterrupt:
        print("fixed action cancelled by operator", file=sys.stderr, flush=True)
        return 130


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
