"""ROS process lifetime shared by the upper-computer entry points."""

import signal


def spin_node(factory, args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        # Do not invalidate the context while a subscription is being taken.
        # The main thread finishes that callback before it destroys ROS handles.
        stopping = True

    previous = {sig: signal.signal(sig, request_stop)
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    executor = None
    initialized = False
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = factory()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while not stopping and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        # A repeated launch/terminal signal must not interrupt cleanup.
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        try:
            if executor is not None:
                executor.shutdown()
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                if initialized and rclpy.ok():
                    rclpy.shutdown(uninstall_handlers=False)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
