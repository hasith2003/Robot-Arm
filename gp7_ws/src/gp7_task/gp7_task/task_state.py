"""Measured-condition state machine for a pick-and-place cycle.

Hardware/simulation adapters call ``advance`` only after their reported result
passes the relevant condition. This prevents a timed script from releasing a
box if a move or grasp failed.
"""
from enum import Enum, auto


class TaskState(Enum):
    HOME = auto()
    PREGRASP = auto()
    DESCEND = auto()
    CLOSE = auto()
    VERIFY_GRASP = auto()
    LIFT = auto()
    TRANSFER = auto()
    LOWER = auto()
    RELEASE = auto()
    VERIFY_PLACE = auto()
    RETREAT = auto()
    DONE = auto()
    FAILED = auto()


class PickPlaceStateMachine:
    """State transitions and their required measured condition."""
    _NEXT = {
        TaskState.HOME: (TaskState.PREGRASP, 'motion_complete'),
        TaskState.PREGRASP: (TaskState.DESCEND, 'motion_complete'),
        TaskState.DESCEND: (TaskState.CLOSE, 'motion_complete'),
        TaskState.CLOSE: (TaskState.VERIFY_GRASP, 'gripper_closed'),
        TaskState.VERIFY_GRASP: (TaskState.LIFT, 'object_attached'),
        TaskState.LIFT: (TaskState.TRANSFER, 'motion_complete'),
        TaskState.TRANSFER: (TaskState.LOWER, 'motion_complete'),
        TaskState.LOWER: (TaskState.RELEASE, 'motion_complete'),
        TaskState.RELEASE: (TaskState.VERIFY_PLACE, 'gripper_open'),
        TaskState.VERIFY_PLACE: (TaskState.RETREAT, 'object_stable_on_target'),
        TaskState.RETREAT: (TaskState.DONE, 'motion_complete'),
    }

    def __init__(self):
        self.state = TaskState.HOME
        self.history = [self.state]

    @property
    def required_condition(self):
        return self._NEXT.get(self.state, (None, None))[1]

    def advance(self, condition):
        if self.state in (TaskState.DONE, TaskState.FAILED):
            raise RuntimeError(f'Cannot advance terminal state {self.state.name}.')
        next_state, required = self._NEXT[self.state]
        if condition != required:
            self.state = TaskState.FAILED
            self.history.append(self.state)
            raise RuntimeError(f'{self.history[-2].name} required {required}, received {condition}.')
        self.state = next_state
        self.history.append(self.state)
        return self.state
