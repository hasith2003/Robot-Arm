import unittest
from gp7_task.task_state import PickPlaceStateMachine, TaskState


class TaskStateTest(unittest.TestCase):
    def test_full_measured_sequence_reaches_done(self):
        machine = PickPlaceStateMachine()
        while machine.state is not TaskState.DONE:
            machine.advance(machine.required_condition)
        self.assertEqual(machine.history[-1], TaskState.DONE)

    def test_wrong_condition_fails_task(self):
        machine = PickPlaceStateMachine()
        with self.assertRaises(RuntimeError):
            machine.advance('object_attached')
        self.assertEqual(machine.state, TaskState.FAILED)
