"""Execute the real main AST with a fake lifecycle; no ROS/Gazebo imports."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / 'src/limo_maze_perception/limo_maze_perception/node.py'


class ExternalShutdown(Exception):
    pass


class LifecycleTests(unittest.TestCase):
    def make_main(self, *, active=True, spin_error=None, construct_error=None,
                  destroy_error=None, init_error=None):
        calls = []

        def init(**kwargs):
            calls.append(('init', kwargs))
            if init_error:
                raise init_error

        def spin(node):
            calls.append('spin')
            if spin_error:
                raise spin_error

        def destroy():
            calls.append('destroy')
            if destroy_error:
                raise destroy_error

        def construct():
            calls.append('construct')
            if construct_error:
                raise construct_error
            return SimpleNamespace(destroy_node=destroy)

        namespace = {'rclpy': SimpleNamespace(init=init, spin=spin, ok=lambda: active,
                         try_shutdown=lambda: calls.append('try_shutdown')),
                     'RedRGBDBaseline': construct, 'ExternalShutdownException': ExternalShutdown}
        parsed = ast.parse(SOURCE.read_text(encoding='utf-8'))
        main = next(item for item in parsed.body if isinstance(item, ast.FunctionDef) and item.name == 'main')
        exec(compile(ast.Module(body=[main], type_ignores=[]), str(SOURCE), 'exec'), namespace)
        return namespace['main'], calls

    def test_normal_return_destroys_then_shuts_down_once(self):
        main, calls = self.make_main()
        main(['--ros-args'])
        self.assertEqual(calls, [('init', {'args': ['--ros-args']}), 'construct', 'spin', 'destroy', 'try_shutdown'])

    def test_ctrl_c_after_signal_handler_shutdown_is_normal(self):
        main, calls = self.make_main(active=False, spin_error=KeyboardInterrupt())
        main()
        self.assertEqual(calls[-2:], ['destroy', 'try_shutdown'])

    def test_external_shutdown_of_context_is_normal(self):
        main, calls = self.make_main(active=False, spin_error=ExternalShutdown())
        main()
        self.assertEqual(calls[-2:], ['destroy', 'try_shutdown'])

    def test_external_exception_with_live_context_is_not_hidden(self):
        error = ExternalShutdown('callback raised unexpectedly')
        main, calls = self.make_main(spin_error=error)
        with self.assertRaises(ExternalShutdown) as caught:
            main()
        self.assertIs(caught.exception, error)
        self.assertEqual(calls[-2:], ['destroy', 'try_shutdown'])

    def test_callback_error_propagates_after_cleanup(self):
        error = RuntimeError('ordinary callback failure')
        main, calls = self.make_main(spin_error=error)
        with self.assertRaises(RuntimeError) as caught:
            main()
        self.assertIs(caught.exception, error)
        self.assertEqual(calls[-2:], ['destroy', 'try_shutdown'])

    def test_constructor_error_propagates_and_still_closes_initialized_context(self):
        error = ValueError('bad detector configuration')
        main, calls = self.make_main(construct_error=error)
        with self.assertRaises(ValueError) as caught:
            main()
        self.assertIs(caught.exception, error)
        self.assertEqual(calls[-2:], ['construct', 'try_shutdown'])
        self.assertNotIn('destroy', calls)

    def test_destroy_error_is_visible_and_context_cleanup_still_runs(self):
        error = RuntimeError('destroy failed')
        main, calls = self.make_main(destroy_error=error)
        with self.assertRaises(RuntimeError) as caught:
            main()
        self.assertIs(caught.exception, error)
        self.assertEqual(calls[-2:], ['destroy', 'try_shutdown'])

    def test_init_failure_is_not_masked_by_uninitialized_shutdown(self):
        error = RuntimeError('init failed')
        main, calls = self.make_main(init_error=error)
        with self.assertRaises(RuntimeError) as caught:
            main()
        self.assertIs(caught.exception, error)
        self.assertEqual(len(calls), 1)


if __name__ == '__main__':
    unittest.main()
