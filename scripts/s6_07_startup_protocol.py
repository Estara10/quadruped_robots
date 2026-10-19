"""Pure protocol decisions shared by the S6-07 ROS helper and offline checks."""

COMMANDS = {
    "STOP": 1,
    "HARD_STOP": 9,
    "NORMAL_SHUTDOWN": 6,
    "START_RL": 3,
    "STAND_STEP": 2,
}


def command_for_publish(token: str, subscriber_count: int) -> tuple[int | None, str | None]:
    """Reject unknown commands and missing subscribers before publish."""
    if token not in COMMANDS:
        return None, "unknown command token"
    if not isinstance(subscriber_count, int) or subscriber_count <= 0:
        return None, "no /control_input subscriber"
    return COMMANDS[token], None


def command_priority(token: str) -> int:
    """No command is queued; safety and stop requests are handled first by callers."""
    if token in {"STOP", "HARD_STOP", "NORMAL_SHUTDOWN"}:
        return 0
    return 1


def publish_outcome(token: str, command: int, error: Exception | None) -> dict:
    """Describe publish-call success only; never claim controller/MuJoCo acceptance."""
    if error is not None:
        return {"event": "FAILED", "token": token,
                "reason": f"{type(error).__name__}: {error}"}
    return {"event": "PUBLISHED", "token": token, "command": command,
            "meaning": "ROS publish call returned; controller acceptance and MuJoCo consumption are separate"}
