#!/usr/bin/python3
import rospy
from threading import Thread, Lock
from serial import Serial
import time
from math import pi
from ctypes import c_short
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

# ---- Wit-motion config commands (write: 0xFF 0xAA ADDR DATAL DATAH) ----
UNLOCK     = b'\xff\xaa\x69\x88\xb5'   # KEY = 0xB588, unlock config registers
SAVE       = b'\xff\xaa\x00\x00\x00'   # save config & restart module
SET_RATE   = b'\xff\xaa\x03'           # register 0x03 = output rate
SET_BAUD   = b'\xff\xaa\x04'           # register 0x04 = serial baud rate
RATE_10HZ  = b'\x06'
RATE_20HZ  = b'\x07'
RATE_50HZ  = b'\x08'
BAUD_9600   = b'\x02'                  # 0x02 = 9600 (factory default)
BAUD_115200 = b'\x06'                  # 0x06 = 115200

SERIAL_BAUD          = 115200
SERIAL_BAUD_FALLBACK = 9600
SERIAL_TIMEOUT       = 0.5

# ---- Data frame: 0x55 <type> + 9 data bytes + checksum = 11 bytes ----
TYPE_ACC   = b'\x51'
TYPE_GYRO  = b'\x52'
TYPE_ANGLE = b'\x53'

buffer_len = 11
XL, XH, YL, YH, ZL, ZH = 2, 3, 4, 5, 6, 7


class IMUConnect:

    def __init__(self, port='/dev/imu_df'):
        rospy.init_node("imu_df_node", anonymous=False)

        self.port = port
        self.read_acc = rospy.get_param('~read_acc', False)

        self.data_lock = Lock()
        self.gyro = (0.0, 0.0, 0.0)   # rad/s (from real gyro packets)
        self.angle = (0.0, 0.0, 0.0)  # rad (roll, pitch, yaw)

        self.imu_pub = rospy.Publisher("imu/data", Imu, queue_size=1)

        self.connection, self.baud = self._setup_serial(port)

        self.start_receive_data()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()

    # ---------------- serial setup / config ----------------

    def _open_serial(self, port, baud):
        return Serial(port=port, baudrate=baud, timeout=SERIAL_TIMEOUT)

    def _write(self, conn, cmd):
        if conn.isOpen():
            conn.write(cmd)
            conn.flush()

    def _send_config(self, conn, register_cmd, value):
        self._write(conn, UNLOCK)
        time.sleep(0.02)
        self._write(conn, register_cmd + value + b'\x00')
        time.sleep(0.02)
        self._write(conn, SAVE)
        time.sleep(0.2)

    def _probe_serial(self, port, baud, duration=1.5):
        """Return True if a valid frame arrives at the given baud."""
        try:
            conn = self._open_serial(port, baud)
            conn.flushInput()
            deadline = time.time() + duration
            while time.time() < deadline:
                if self._read_frame(conn) is not None:
                    conn.close()
                    return True
            conn.close()
        except Exception as e:
            rospy.logerr("IMU: probe at %d baud failed: %s", baud, e)
        return False

    def _setup_serial(self, port):
        # 1) already configured at target baud?
        if self._probe_serial(port, SERIAL_BAUD):
            rospy.loginfo("IMU: sensor OK at %d baud (already configured)", SERIAL_BAUD)
            return self._open_serial(port, SERIAL_BAUD), SERIAL_BAUD

        rospy.logwarn("IMU: no data at %d baud -> configuring sensor (20Hz / %d baud)...",
                      SERIAL_BAUD, SERIAL_BAUD)
        # 2) configure from factory default baud (9600)
        try:
            conn = self._open_serial(port, SERIAL_BAUD_FALLBACK)
            conn.flushInput()
            self._send_config(conn, SET_RATE, RATE_20HZ)
            self._send_config(conn, SET_BAUD, BAUD_115200)
            conn.close()
            time.sleep(1.0)   # module restarts at the new baud rate
        except Exception as e:
            rospy.logerr("IMU: config send failed: %s", e)

        # 3) verify at new baud
        if self._probe_serial(port, SERIAL_BAUD):
            rospy.loginfo("IMU: sensor configured OK at %d baud", SERIAL_BAUD)
            return self._open_serial(port, SERIAL_BAUD), SERIAL_BAUD

        # 4) fallback: keep 9600 baud, only bump rate to 20Hz
        rospy.logerr("IMU: %d baud config failed -> fallback to %d baud / 20Hz",
                     SERIAL_BAUD, SERIAL_BAUD_FALLBACK)
        try:
            conn = self._open_serial(port, SERIAL_BAUD_FALLBACK)
            conn.flushInput()
            self._send_config(conn, SET_RATE, RATE_20HZ)
            conn.close()
            time.sleep(0.5)
        except Exception as e:
            rospy.logerr("IMU: fallback rate config failed: %s", e)
        if self._probe_serial(port, SERIAL_BAUD_FALLBACK):
            return self._open_serial(port, SERIAL_BAUD_FALLBACK), SERIAL_BAUD_FALLBACK
        return self._open_serial(port, SERIAL_BAUD_FALLBACK), SERIAL_BAUD_FALLBACK

    # ---------------- frame reading ----------------

    def _read_frame(self, conn):
        """Return one valid 11-byte frame, or None on timeout/checksum failure."""
        b0 = conn.read(1)
        while b0 and b0 != b'\x55':
            b0 = conn.read(1)
        if not b0:
            return None
        typ = conn.read(1)
        if typ not in (TYPE_ACC, TYPE_GYRO, TYPE_ANGLE):
            return None
        rest = conn.read(9)
        if len(rest) != 9:
            return None
        frame = b0 + typ + rest
        if (sum(frame[:10]) & 0xFF) != frame[10]:
            return None
        return frame

    def start_receive_data(self):
        Thread(target=self.reading_thread, daemon=True).start()

    def reading_thread(self):
        rospy.loginfo("IMU: reading thread started (%d baud)", self.baud)
        while not rospy.is_shutdown():
            try:
                frame = self._read_frame(self.connection)
                if frame is None:
                    continue
                typ = frame[1:2]
                if typ == TYPE_ANGLE:
                    roll  = self.get_angle(frame[XL], frame[XH])
                    pitch = self.get_angle(frame[YL], frame[YH])
                    yaw   = self.get_angle(frame[ZL], frame[ZH])
                    self.on_angle(roll, pitch, yaw)
                elif typ == TYPE_GYRO:
                    gx = self.get_gyro(frame[XL], frame[XH])
                    gy = self.get_gyro(frame[YL], frame[YH])
                    gz = self.get_gyro(frame[ZL], frame[ZH])
                    self.on_gyro(gx, gy, gz)
            except Exception:
                rospy.logwarn_throttle(5.0, "IMU: serial error, reconnecting...")
                try:
                    self.connection.close()
                except Exception:
                    pass
                time.sleep(0.5)
                try:
                    self.connection = self._open_serial(self.port, self.baud)
                except Exception as e:
                    rospy.logerr("IMU: reconnect failed: %s", e)
                    time.sleep(1.0)

    # ---------------- data handling ----------------

    def on_gyro(self, gx_dps, gy_dps, gz_dps):
        to_rad = pi / 180.0
        with self.data_lock:
            self.gyro = (gx_dps * to_rad, gy_dps * to_rad, gz_dps * to_rad)

    def on_angle(self, roll, pitch, yaw):
        with self.data_lock:
            self.angle = (roll, pitch, yaw)
            gyro = self.gyro
        self.publish_imu(roll, pitch, yaw, gyro)

    def publish_imu(self, roll, pitch, yaw, gyro):
        # Keep existing yaw convention: sensor Z in [0,2pi) -> ROS [-pi, pi]
        output_yaw = yaw - pi

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation = self.quaternion_from_RPY(0.0, 0.0, output_yaw)
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_noises = pow(0.00017, 2)
        imu_msg.orientation_covariance = [1e3, 0, 0, 0, 1e3, 0, 0, 0, imu_noises]
        self.imu_pub.publish(imu_msg)

    # ---------------- raw conversion ----------------

    def get_acc(self, data_low, data_high):
        low_byte = data_low
        high_byte = data_high
        result = high_byte << 8 | low_byte
        result = c_short(result).value / 32768.000 * 16.000 * 9.806
        return result

    def get_gyro(self, data_low, data_high):
        low_byte = data_low
        high_byte = data_high
        result = high_byte << 8 | low_byte
        result = c_short(result).value / 32768.000 * 2000.000
        return result

    def get_angle(self, data_low, data_high):
        low_byte = data_low
        high_byte = data_high
        result = high_byte << 8 | low_byte
        result = c_short(result).value / 32768.000 * pi
        return result

    def quaternion_from_RPY(self, roll, pitch, yaw):
        tmp = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(tmp[0], tmp[1], tmp[2], tmp[3])

    def flush(self):
        self.connection.flushInput()
        self.connection.flushOutput()

    def open(self):
        if not self.connection.isOpen():
            self.connection.open()

    def close(self):
        self.flush()
        self.connection.close()

    def send(self, command):
        self.open()
        if self.connection.writable:
            self.connection.write(command)


if __name__ == "__main__":
    serial = IMUConnect()