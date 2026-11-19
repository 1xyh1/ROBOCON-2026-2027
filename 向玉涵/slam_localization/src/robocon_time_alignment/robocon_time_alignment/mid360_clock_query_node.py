"""Nonblocking, read-only MID-360 clock telemetry, independent of SDK sockets."""

import json
import socket
import time

from rclpy.node import Node
from std_msgs.msg import String

from robocon_core.mid360_protocol import parse_clock_response, query_clock_packet
from robocon_core.ros_runtime import spin_node


class Mid360ClockQueryNode(Node):
    def __init__(self):
        super().__init__("mid360_clock_query")
        self.declare_parameter("lidar_ip", "192.168.1.139")
        self.declare_parameter("host_ip", "0.0.0.0")
        self.peer = (str(self.get_parameter("lidar_ip").value), 56100)
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.connection.bind((str(self.get_parameter("host_ip").value), 0))
        self.connection.setblocking(False)
        self.sequence = time.monotonic_ns() & 0xFFFFFFFF
        self.pending = None
        self.last_good_monotonic = None
        self.publisher = self.create_publisher(String, "/livox/clock_device", 10)
        self.create_timer(1.0, self._send)
        self.create_timer(0.02, self._poll)

    def _publish(self, fields):
        fields["received_utc_ns"] = self.get_clock().now().nanoseconds
        self.publisher.publish(String(data=json.dumps(fields)))

    def _send(self):
        if self.pending is not None:
            return
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        started = time.monotonic()
        try:
            self.connection.sendto(query_clock_packet(self.sequence), self.peer)
        except OSError as error:
            self._publish(dict(valid=False, error=str(error)))
            return
        self.pending = (self.sequence, started)

    def _poll(self):
        for _ in range(32):
            try:
                packet, peer = self.connection.recvfrom(1500)
            except BlockingIOError:
                break
            except OSError as error:
                self.pending = None
                self._publish(dict(valid=False, error=str(error)))
                return
            if self.pending is None or peer != self.peer:
                continue
            sequence, started = self.pending
            # A late response must not invalidate a different outstanding query.
            if len(packet) >= 8 and int.from_bytes(packet[4:8], "little") != sequence:
                continue
            try:
                fields = parse_clock_response(packet, sequence)
            except ValueError as error:
                fields = dict(valid=False, error=str(error))
            else:
                fields.update(valid=True, query_round_trip_s=time.monotonic() - started)
                self.last_good_monotonic = time.monotonic()
            self.pending = None
            self._publish(fields)
        if self.pending is not None and time.monotonic() - self.pending[1] >= 0.8:
            self.pending = None
            # A single lost read-only UDP reply is not proof of clock loss.
            # Both native and ROS gates already reject the previous evidence
            # once it is older than 2.5 s; keep that same failure boundary.
            if (self.last_good_monotonic is None or
                    time.monotonic() - self.last_good_monotonic >= 2.5):
                self._publish(dict(valid=False, error="clock_query_timeout"))

    def destroy_node(self):
        self.connection.close()
        return super().destroy_node()


def main(args=None):
    spin_node(Mid360ClockQueryNode, args)
