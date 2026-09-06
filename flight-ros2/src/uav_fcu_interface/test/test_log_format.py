import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).parents[1]
FCU_PATH = ROOT / 'uav_fcu_interface' / 'fcu_interface_mavros_node.py'
MISSION_PATH = (
    ROOT.parent
    / 'uav_mission_manager'
    / 'uav_mission_manager'
    / 'mission_manager_node.py'
)
PAYLOAD_PATH = (
    ROOT.parent / 'uav_payload' / 'uav_payload' / 'payload_monitor_node.py'
)


def source(path):
    return path.read_text(encoding='utf-8')


def function_source(path, name):
    text = source(path)
    tree = ast.parse(text)
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef))
    function = next(
        item
        for item in node.body
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
        and item.name == name
    )
    return ast.get_source_segment(text, function)


class LogFormatTest(unittest.TestCase):
    def test_fcu_periodic_status_uses_status_prefix(self):
        body = function_source(FCU_PATH, 'print_status')
        self.assertIn("self._prefix('STATUS')", body)

    def test_waypoint_reached_uses_fcu_prefix(self):
        body = function_source(FCU_PATH, 'waypoint_reached_callback')
        self.assertIn("self._prefix('FCU')", body)
        self.assertIn('waypoint reached seq=', body)

    def test_plan_logs_include_shared_task_context(self):
        body = function_source(FCU_PATH, 'handle_goto_global')
        self.assertIn("self._prefix('PLAN')", body)
        self.assertIn('planning started', body)

    def test_safety_rejections_include_reason(self):
        body = function_source(FCU_PATH, 'handle_goto_global')
        self.assertIn("self._prefix('SAFETY', 'REJECTED')", body)
        self.assertIn('reason=', body)

    def test_composite_upload_requires_auto_without_changing_mode(self):
        entry_body = function_source(FCU_PATH, 'handle_goto_global')
        composite_body = function_source(
            FCU_PATH, 'handle_goto_global_composite_async'
        )

        self.assertIn("mode_before_upload != 'AUTO'", entry_body)
        self.assertIn('FcuInterface will not change flight mode', entry_body)
        self.assertNotIn('self.set_mode(', entry_body)
        self.assertNotIn('self.set_mode(', composite_body)
        self.assertIn('mode_before_set_current', composite_body)
        self.assertIn('mode_after_set_current', composite_body)
        self.assertIn(
            'no automatic mode change or LOITER rollback was attempted',
            composite_body,
        )

    def test_legacy_temporary_task_island_is_removed(self):
        text = source(FCU_PATH)
        for marker in (
            "mission_type == 'TEMPORARY'",
            'upload_temporary_auto_mission',
            'restart_temporary_auto_mission',
            'execute_refly_then_restart_task_mission',
            'restore_original_auto_mission',
            '/fcu/resume_auto',
            'self.set_mode(',
        ):
            self.assertNotIn(marker, text)

    def test_no_task_uses_none_not_python_none(self):
        self.assertIn("self.active_task_id = 'none'", source(FCU_PATH))

    def test_boolean_formatter_is_lowercase(self):
        body = function_source(FCU_PATH, '_bool')
        self.assertIn('.lower()', body)

    def test_payload_core_prefix_is_preserved(self):
        text = source(PAYLOAD_PATH)
        self.assertIn("f'[PAYLOAD][task={self.monitor.task_id}]'", text)
        self.assertIn("f'[state={state or self.monitor.state}]'", text)

    def test_status_timers_remain_throttled(self):
        self.assertIn('self.create_timer(\n            5.0', source(FCU_PATH))
        self.assertIn(
            'self.create_timer(2.0, self.print_state)',
            source(MISSION_PATH),
        )


if __name__ == '__main__':
    unittest.main()
