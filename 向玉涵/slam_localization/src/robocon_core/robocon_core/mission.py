"""Top-level BR mission state machine with resumable, verified checkpoints."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field, fields
from enum import Enum
import json
import os
from pathlib import Path
from typing import Any
from uuid import uuid4


class CheckpointPersistenceError(RuntimeError):
    """The transition was not durably committed; motion must stay disabled."""


class CheckpointCorruptError(ValueError):
    """The on-disk state cannot be trusted and must be reconciled manually."""


class MissionState(str, Enum):
    BOOT = "boot"
    SELF_CHECK = "self_check"
    READY = "ready"
    ARMED_WAIT = "armed_wait"
    RUNNING = "running"
    SAFE_HOLD = "safe_hold"
    RELOCALIZE = "relocalize"
    RECONCILE = "reconcile"
    RETRY_REQUIRED = "retry_required"
    AUTHORIZED_RETRY = "authorized_retry"
    FINISHED = "finished"
    FAULT = "fault"


class MissionPhase(str, Enum):
    IDLE = "idle"
    RECEIVE_BLOCK = "receive_block"
    NAVIGATE_TO_BUILD = "navigate_to_build"
    OBSERVE_BUILD_SPOT = "observe_build_spot"
    PLACE_BLOCK = "place_block"
    VERIFY_PLACEMENT = "verify_placement"
    RECEIVE_MUSTIKA = "receive_mustika"
    NAVIGATE_TO_PILLAR = "navigate_to_pillar"
    PLACE_MUSTIKA = "place_mustika"
    CONTINUE_SCORING = "continue_scoring"


@dataclass
class MissionCheckpoint:
    schema_version: int = 1
    match_id: str = ""
    state: str = MissionState.BOOT.value
    phase: str = MissionPhase.IDLE.value
    floor: str = "ground"
    last_verified_action: str = ""
    active_action_id: str = ""
    expected_payload: list[str] = field(default_factory=list)
    entered_l1: bool = False
    mandate_achieved: bool = False
    start_sequence: int = 0
    context: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ExecutiveCommand:
    action_id: str
    action_type: str
    payload: dict[str, Any]


class MissionController:
    """Deterministic executive; ROS Actions execute the returned commands."""

    def __init__(self, checkpoint_path: str | Path | None = None) -> None:
        self.checkpoint_path = Path(checkpoint_path) if checkpoint_path else None
        self.checkpoint = MissionCheckpoint()
        self.state = MissionState.BOOT
        self._committed_checkpoint = deepcopy(self.checkpoint)
        self._committed_state = self.state

    def begin_self_check(self) -> None:
        self._move(MissionState.SELF_CHECK, {MissionState.BOOT})

    def set_ready(self, match_id: str) -> None:
        self._require_state(MissionState.READY, {MissionState.SELF_CHECK})
        if not isinstance(match_id, str) or not match_id.strip():
            raise ValueError("match_id must be nonempty")
        self.checkpoint.match_id = match_id
        self._move(MissionState.READY, {MissionState.SELF_CHECK})

    def arm(self) -> None:
        self._move(MissionState.ARMED_WAIT, {MissionState.READY})

    def accept_start(self, sequence: int) -> bool:
        if self.state != MissionState.ARMED_WAIT or sequence <= self.checkpoint.start_sequence:
            return False
        self.checkpoint.start_sequence = sequence
        self.checkpoint.context["stop_confirmed"] = False
        self._move(MissionState.RUNNING, {MissionState.ARMED_WAIT})
        return True

    def issue(self, action_id: str, action_type: str, phase: MissionPhase, **payload: Any) -> ExecutiveCommand:
        if self.state != MissionState.RUNNING:
            raise RuntimeError("mission actions may start only while running")
        if self.checkpoint.active_action_id:
            raise RuntimeError("another mission action is still active")
        if not action_id or not action_type:
            raise ValueError("action_id and action_type must be nonempty")
        self.checkpoint.active_action_id = action_id
        self.checkpoint.phase = phase.value
        self.checkpoint.context["active_command"] = {"action_type": action_type, "payload": payload}
        self.checkpoint.context["stop_confirmed"] = False
        self._save()
        return ExecutiveCommand(action_id=action_id, action_type=action_type, payload=payload)

    def verify_action(self, action_id: str, **facts: Any) -> None:
        if self.state != MissionState.RUNNING:
            raise RuntimeError("interrupted actions must be reconciled from observations")
        if not action_id or action_id != self.checkpoint.active_action_id:
            raise ValueError("completion does not match the active action")
        self.checkpoint.last_verified_action = action_id
        self.checkpoint.active_action_id = ""
        self.checkpoint.context.pop("active_command", None)
        self.checkpoint.context.setdefault("facts", {}).update(facts)
        self._save()

    def localization_lost(self) -> str:
        self._require_state(MissionState.SAFE_HOLD, {MissionState.RUNNING})
        cancelled = self.checkpoint.active_action_id
        self.checkpoint.context["stop_confirmed"] = False
        self._move(MissionState.SAFE_HOLD, {MissionState.RUNNING})
        return cancelled

    def begin_relocalization(self) -> None:
        self._require_state(MissionState.RELOCALIZE, {MissionState.SAFE_HOLD, MissionState.AUTHORIZED_RETRY})
        if not self.checkpoint.context.get("stop_confirmed", False):
            raise RuntimeError("waiting for executor stop acknowledgement")
        self._move(MissionState.RELOCALIZE, {MissionState.SAFE_HOLD, MissionState.AUTHORIZED_RETRY})

    def localization_recovered(self, floor: str, *, evidence_id: str) -> None:
        self._require_state(MissionState.RECONCILE, {MissionState.RELOCALIZE})
        if not floor or not evidence_id:
            raise ValueError("global localization floor and evidence_id are required")
        self.checkpoint.floor = floor
        self.checkpoint.context["global_localization_evidence"] = evidence_id
        self._move(MissionState.RECONCILE, {MissionState.RELOCALIZE})

    def resume_after_reconcile(
        self,
        *,
        payload: list[str],
        action_had_effect: bool,
        evidence_id: str = "",
    ) -> None:
        # Validate before touching the checkpoint: rejected calls must not
        # erase the active action or promote it to a verified completion.
        self._require_state(MissionState.RUNNING, {MissionState.RECONCILE})
        if not evidence_id:
            raise ValueError("physical reconciliation evidence_id is required")
        if not self.checkpoint.context.get("stop_confirmed", False):
            raise RuntimeError("waiting for executor stop acknowledgement")
        if not self.checkpoint.context.get("global_localization_evidence"):
            raise RuntimeError("fresh global localization evidence is required")
        if action_had_effect and not self.checkpoint.active_action_id:
            raise ValueError("no interrupted action exists to mark as completed")
        self.checkpoint.expected_payload = list(payload)
        if action_had_effect and self.checkpoint.active_action_id:
            self.checkpoint.last_verified_action = self.checkpoint.active_action_id
        self.checkpoint.context["last_reconcile"] = {
            "action_had_effect": action_had_effect,
            "evidence_id": evidence_id,
        }
        self.checkpoint.active_action_id = ""
        self.checkpoint.context.pop("active_command", None)
        self.checkpoint.context["stop_confirmed"] = False
        self._move(MissionState.RUNNING, {MissionState.RECONCILE})

    def require_retry(self) -> str:
        allowed = {MissionState.RUNNING, MissionState.SAFE_HOLD, MissionState.FAULT,
                   MissionState.RELOCALIZE, MissionState.RECONCILE}
        self._require_state(MissionState.RETRY_REQUIRED, allowed)
        cancelled = self.checkpoint.active_action_id
        self.checkpoint.context["stop_confirmed"] = False
        self._move(
            MissionState.RETRY_REQUIRED,
            allowed,
        )
        return cancelled

    def authorize_retry(self) -> None:
        self._move(MissionState.AUTHORIZED_RETRY, {MissionState.RETRY_REQUIRED})

    def mark_entered_l1(self) -> None:
        self.checkpoint.entered_l1 = True
        self.checkpoint.floor = "l1"
        self._save()

    def mark_mandate_achieved(self) -> None:
        self.checkpoint.mandate_achieved = True
        self._save()

    def finish(self) -> None:
        if self.state == MissionState.FINISHED:
            return
        if self.checkpoint.start_sequence or self.checkpoint.active_action_id:
            if not self.checkpoint.context.get("stop_confirmed", False):
                raise RuntimeError("finish requires executor stop acknowledgement")
        if self.checkpoint.active_action_id:
            self.checkpoint.context["aborted_action_id"] = self.checkpoint.active_action_id
        self.checkpoint.active_action_id = ""
        self.checkpoint.context.pop("active_command", None)
        self._move(MissionState.FINISHED, set(MissionState) - {MissionState.FINISHED})

    def confirm_stopped(self, action_id: str) -> None:
        # The ROS adapter separately verifies the stop request ID. An action
        # may complete between cancel publication and acknowledgement.
        if self.checkpoint.active_action_id and action_id != self.checkpoint.active_action_id:
            raise ValueError("stop acknowledgement does not match active action")
        self.checkpoint.context["stop_confirmed"] = True
        self._save()

    def fault(self, reason: str) -> str:
        if self.state == MissionState.FINISHED:
            return ""  # Preserve the terminal match record after late faults.
        cancelled = self.checkpoint.active_action_id
        self.checkpoint.context["fault_reason"] = reason
        self.checkpoint.context["stop_confirmed"] = False
        self.state = MissionState.FAULT
        self._save()
        return cancelled

    def load_for_reconciliation(self) -> MissionCheckpoint:
        if not self.checkpoint_path or not self.checkpoint_path.exists():
            return self.checkpoint
        try:
            data = json.loads(self.checkpoint_path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("checkpoint root is not an object")
            version = data.get("schema_version", 0)
            if version not in (0, 1):
                raise ValueError(f"unsupported checkpoint schema {version!r}")
            allowed = {item.name for item in fields(MissionCheckpoint)}
            unknown = set(data) - allowed
            if unknown or not {"state", "phase", "context"} <= set(data):
                raise ValueError(f"checkpoint fields invalid: unknown={sorted(unknown)}")
            data["schema_version"] = 1  # Explicit migration from the old schema.
            loaded = MissionCheckpoint(**data)
            if not isinstance(loaded.context, dict) or not isinstance(loaded.expected_payload, list):
                raise ValueError("checkpoint context or payload has the wrong type")
            previous = MissionState(loaded.state)
        except (OSError, ValueError, TypeError) as exc:
            raise CheckpointCorruptError(
                f"checkpoint at {self.checkpoint_path} is invalid; preserve it for manual reconciliation: {exc}"
            ) from exc
        self.checkpoint = loaded
        self.state = previous
        self._committed_checkpoint = deepcopy(loaded)
        self._committed_state = previous
        if previous == MissionState.FINISHED:
            self.state = MissionState.FINISHED
        elif previous in {
            MissionState.BOOT,
            MissionState.SELF_CHECK,
            MissionState.READY,
            MissionState.ARMED_WAIT,
        }:
            # A restart before the official start must repeat health checks and
            # arm again; it must not turn an old ARMED_WAIT into RUNNING.
            self.state = MissionState.SELF_CHECK
        elif previous in {MissionState.RETRY_REQUIRED, MissionState.FAULT}:
            self.state = previous
        else:
            # Any interrupted physical action is reconciled from observations,
            # never replayed from the saved program counter.
            self.state = MissionState.SAFE_HOLD
        if self.state != MissionState.FINISHED:
            self.checkpoint.context["stop_confirmed"] = False
            self.checkpoint.context.pop("global_localization_evidence", None)
        self.checkpoint.state = self.state.value
        self._save()
        return self.checkpoint

    def _move(self, target: MissionState, allowed: set[MissionState]) -> None:
        self._require_state(target, allowed)
        self.state = target
        self._save()

    def _require_state(self, target: MissionState, allowed: set[MissionState]) -> None:
        if self.state not in allowed:
            raise RuntimeError(f"invalid transition {self.state.value} -> {target.value}")

    def _save(self) -> None:
        self.checkpoint.state = self.state.value
        if not self.checkpoint_path:
            self._committed_checkpoint = deepcopy(self.checkpoint)
            self._committed_state = self.state
            return
        temporary = self.checkpoint_path.with_name(
            self.checkpoint_path.name + ".tmp-" + uuid4().hex)
        try:
            self.checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
            with temporary.open("x", encoding="utf-8") as stream:
                json.dump(asdict(self.checkpoint), stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.checkpoint_path)
        except (OSError, TypeError, ValueError) as exc:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            self.checkpoint = deepcopy(self._committed_checkpoint)
            self.state = self._committed_state
            raise CheckpointPersistenceError(
                f"checkpoint write failed at {self.checkpoint_path}: {exc}"
            ) from exc
        self._committed_checkpoint = deepcopy(self.checkpoint)
        self._committed_state = self.state
