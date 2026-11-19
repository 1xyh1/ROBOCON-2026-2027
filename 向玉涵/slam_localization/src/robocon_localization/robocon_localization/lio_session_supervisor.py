"""Own the LIO child process and publish its assigned epoch while it lives."""
import os
from pathlib import Path
import signal
import subprocess

from rclpy.node import Node
from std_msgs.msg import UInt64

from robocon_core.ros_runtime import spin_node


class LioSessionSupervisor(Node):
    def __init__(self):
        super().__init__('lio_session_supervisor')
        self.declare_parameter('frontend_executable', '')
        self.declare_parameter('config_file', '')
        self.declare_parameter('odom_epoch', -1)
        executable = Path(self.get_parameter('frontend_executable').value)
        config = Path(self.get_parameter('config_file').value)
        epoch = self.get_parameter('odom_epoch').value
        if not executable.is_absolute() or not os.access(executable, os.X_OK) or not config.is_file():
            raise ValueError('An executable frontend and existing parameter file are required')
        if type(epoch) is not int or epoch < 1:
            raise ValueError('A launch-assigned odometry epoch is required')
        self.epoch = UInt64(data=epoch)
        self.publisher = self.create_publisher(UInt64, '/localization/odom_epoch', 10)
        sim_time = str(self.get_parameter('use_sim_time').value).lower()
        self.child = subprocess.Popen([
            str(executable), '--ros-args', '--params-file', str(config),
            '-r', '__node:=laser_mapping', '-p', 'use_sim_time:=' + sim_time,
        ], start_new_session=True)
        self.create_timer(0.2, self._heartbeat)

    def _heartbeat(self):
        code = self.child.poll()
        if code is not None:
            raise RuntimeError(f'LIO session {self.epoch.data} ended: frontend exit {code}')
        self.publisher.publish(self.epoch)

    def destroy_node(self):
        if self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
            try:
                self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGTERM)
                try:
                    self.child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(self.child.pid, signal.SIGKILL)
                    self.child.wait()
        return super().destroy_node()


def main(args=None):
    spin_node(LioSessionSupervisor, args)
