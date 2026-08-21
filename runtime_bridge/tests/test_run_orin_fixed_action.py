from runtime_bridge.apps.run_orin_fixed_action import run
from runtime_bridge.orin_follow_client import FixedActionResult


class _Client:
    def __init__(self, host, port):
        self.endpoint = (host, port)
        self.calls = []

    def run_fixed_action(self, behavior, *, feedback_callback):
        self.calls.append(behavior)
        feedback_callback(
            type(
                "Feedback",
                (),
                {
                    "step_index": 0,
                    "step_label": "open_bucket",
                    "phase": "settling",
                    "max_error": 0.01,
                    "action_datagrams": 4,
                },
            )()
        )
        return FixedActionResult(
            behavior=behavior,
            outcome="SUCCEEDED",
            reason_code="SEQUENCE_COMPLETED",
            message="done",
            final_step_index=0,
            final_step_label="open_bucket",
            final_max_error=0.01,
            quiescence_confirmed=True,
            action_datagrams=8,
        )


def test_fixed_action_cli_runs_one_orin_behavior_and_requires_quiescence(capsys):
    clients = []

    def factory(host, port):
        client = _Client(host, port)
        clients.append(client)
        return client

    result = run(
        ["ExecuteDump", "--host", "192.168.50.2", "--port", "18083"],
        client_factory=factory,
    )

    assert result == 0
    assert clients[0].endpoint == ("192.168.50.2", 18083)
    assert clients[0].calls == ["ExecuteDump"]
    assert "SEQUENCE_COMPLETED" in capsys.readouterr().out


def test_fixed_action_cli_rejects_nonquiescent_success(capsys):
    class _UnsafeClient(_Client):
        def run_fixed_action(self, behavior, *, feedback_callback):
            result = super().run_fixed_action(
                behavior, feedback_callback=feedback_callback
            )
            return FixedActionResult(
                **{**result.__dict__, "quiescence_confirmed": False}
            )

    result = run(
        ["ExecuteDump", "--host", "192.168.50.2"],
        client_factory=_UnsafeClient,
    )

    assert result == 2
    assert "quiescence" in capsys.readouterr().err
