import math
import serial
from threading import Thread

import rclpy
from rclpy.node import Node, SetParametersResult
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('LidarNode has been started.')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 230400)
        self.declare_parameter('frame_id', 'lidar_frame')
        self.declare_parameter('period_try_reopen', 1.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.period_try_reopen = self.get_parameter('period_try_reopen').value

        self.conn = None
        self.num_points_scan = 240
        self.ranges = [float('inf')] * self.num_points_scan
        self.intensities = [0.0] * self.num_points_scan
        self.last_angle_start = None

        self.try_reopen_serial = self.create_timer(
            timer_period_sec = self.period_try_reopen,
            callback = self.initialize_serial_connection,
            autostart = True,
        )

        self.add_on_set_parameters_callback(
            self.update_parameters
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10,
        )

        self.reader_thread = Thread(
            target=self.serial_reader_loop,
            daemon=True,
        )
        self.reader_thread.start()


    def update_parameters(self, params):
        for param in params:
            if param.name == 'serial_port' and param.value != self.serial_port:
                self.serial_port = param.value
                self.try_reopen_serial.reset()
                self.get_logger().info(f'Updated \"serial_port\" to \"{self.serial_port}\".')

            elif param.name == 'serial_baud' and param.value != self.serial_baud:
                self.serial_baud = param.value
                self.try_reopen_serial.reset()
                self.get_logger().info(f'Updated \"serial_baud\" to \"{self.serial_baud}\".')

            elif param.name == 'frame_id' and param.value != self.frame_id:
                self.frame_id = param.value
                self.get_logger().info(f'Updated \"frame_id\" to \"{self.frame_id}\".')

            elif param.name == 'period_try_reopen' and param.value != self.period_try_reopen:
                self.period_try_reopen = param.value
                autostart = not self.try_reopen_serial.is_canceled()

                if autostart:
                    self.try_reopen_serial.cancel()

                self.try_reopen_serial = self.create_timer(
                    timer_period_sec = self.period_try_reopen,
                    callback = self.initialize_serial_connection,
                    autostart = autostart,
                )

                self.get_logger().info(f'Updated \"period_try_reopen\" to \"{self.period_try_reopen}\".')

        return SetParametersResult(successful=True)


    def initialize_serial_connection(self):
        try:
            if self.conn is not None:
                try:
                    self.conn.close()
                except Exception:
                    pass

            self.conn = serial.Serial(
                port = self.serial_port,
                baudrate = self.serial_baud,
                timeout = 1,
            )

            if self.conn.is_open:

                if not self.try_reopen_serial.is_canceled():
                    self.try_reopen_serial.cancel()

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')


    def extract_packets(self):
        while True:
            start = self.buffer.find(b"\xAA\x00")

            if start == -1:
                if len(self.buffer) > 2:
                    del self.buffer[:-2]
                return

            if start > 0:
                del self.buffer[:start]

            if len(self.buffer) < 8:
                return

            param_len = (
                (self.buffer[6] << 8)
                | self.buffer[7]
            )

            packet_len = 8 + param_len + 2

            if len(self.buffer) < packet_len:
                return

            yield bytes(self.buffer[:packet_len])

            del self.buffer[:packet_len]


    def serial_reader_loop(self):
        self.get_logger().info('Start thread.')

        self.buffer = bytearray()

        while rclpy.ok():
            if not self.try_reopen_serial.is_canceled():
                self.buffer.clear()
                continue

            try:
                if self.conn is None or not self.conn.is_open:
                    continue

                data = self.conn.read_all()

            except serial.SerialException:
                self.get_logger().error('Error in read serial. Try reopen.')

                if self.try_reopen_serial.is_canceled():
                    self.try_reopen_serial.reset()
                self.buffer.clear()
                continue

            if not data:
                continue

            self.buffer.extend(data)

            for packet in self.extract_packets():

                frame_command = packet[5]

                if frame_command == 0xAD:

                    param_len = (
                        (packet[6] << 8)
                        | packet[7]
                    )

                    frame_data = packet[8 : 8 + param_len]

                    checksum_given = (
                        (packet[8 + param_len] << 8)
                        | packet[8 + param_len + 1]
                    )

                    checksum_calc = sum(packet[:8 + param_len]) & 0xFFFF

                    if checksum_calc != checksum_given:
                        continue

                    angle_start = (
                        (frame_data[3] << 8)
                        | frame_data[4]
                    ) * 0.01

                    num_points = (param_len - 9) // 3

                    start_index = int(angle_start / 1.5)

                    if (
                        self.last_angle_start is not None and
                        angle_start < self.last_angle_start
                    ):

                        msg = LaserScan()

                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = self.frame_id

                        msg.angle_min = 0.0
                        msg.angle_max = math.radians(358.5)
                        msg.angle_increment = math.radians(1.5)

                        msg.time_increment = 0.0
                        msg.scan_time = 0.0

                        msg.range_min = 0.05
                        msg.range_max = 8.0

                        msg.ranges = self.ranges.copy()
                        msg.intensities = self.intensities.copy()

                        self.scan_pub.publish(msg)

                    self.last_angle_start = angle_start

                    offset = 9

                    for i in range(num_points):

                        distance_raw = (
                            (frame_data[offset] << 8)
                            | frame_data[offset + 1]
                        )

                        distance = distance_raw * 0.00025

                        quality = float(
                            frame_data[offset + 2]
                        )

                        index = start_index + i

                        if 0 <= index < self.num_points_scan:

                            if distance > 0:
                                self.ranges[index] = distance
                            else:
                                self.ranges[index] = float('inf')

                            self.intensities[index] = quality

                        offset += 3

    def destroy_node(self):
        self.get_logger().info('LidarNode is shutting down.')
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass

        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.1)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        lidar_node = LidarNode()
        rclpy.spin(lidar_node)

    except KeyboardInterrupt:
        pass

    else:
        lidar_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
