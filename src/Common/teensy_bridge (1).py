# teensy_bridge.py
# Handles all UART communication between Pi and Teensy
#
# Pi → Teensy: "S<rpm>,<steer>\n"
# Teensy → Pi: "T<rpm>,<ticks>,<heading>\n"

import serial
import threading
import time


class TeensyBridge:
    """
    Sends commands to Teensy and receives telemetry.
    Runs receive loop in background thread.
    Call get_telemetry() to read latest Teensy state.
    Call send_command() to drive motor and servo.
    """

    PORT = '/dev/ttyAMA0'
    BAUD = 460800

    def __init__(self):
        self._serial  = None
        self._running = False
        self._lock    = threading.Lock()
        self._zero_requested_at = None   

        # Latest telemetry from Teensy
        self._telemetry = {
            'actual_rpm': 0.0,
            'total_ticks': 0,
            'heading': 0.0,

            'imu_raw': 0.0,

            'imu_sys_cal': 0,
            'imu_gyro_cal': 0,
            'imu_accel_cal': 0,
            'imu_mag_cal': 0,

            'last_update': 0.0,
        }

        self._rx_buf   = ''
        self._msg_count = 0

    def start(self):
        self._serial  = serial.Serial(
            self.PORT, self.BAUD,
            timeout=0.1)
        self._running = True

        self._thread = threading.Thread(
            target=self._receive_loop,
            daemon=True)
        self._thread.start()

        # Wait for first telemetry
        print("Waiting for Teensy...", end='', flush=True)
        timeout = time.time() + 5
        while self._telemetry['last_update'] == 0:
            if time.time() > timeout:
                print("\nWARNING: No telemetry from Teensy")
                return False
            print('.', end='', flush=True)
            time.sleep(0.1)

        print(f" connected. Heading={self._telemetry['heading']:.1f}°")
        return True

    def stop(self):
        self._running = False
        # Send stop command before closing
        self.send_command(0, 0)
        time.sleep(0.1)
        if self._serial:
            self._serial.close()

    def send_command(self, rpm: float, steer_deg: float):
        """
        Send speed and steering to Teensy.
        rpm:       -240 to +240
        steer_deg: -30 to +30
                   negative = left, positive = right
        """
        if not self._serial:
            return
        msg = f"S{rpm:.1f},{steer_deg:.1f}\n"
        try:
            self._serial.write(msg.encode())
        except Exception as e:
            print(f"UART TX error: {e}")
            
    def zero_imu(self):
        """
        Reset IMU zero on the Teensy.
        """
        if not self._serial:
            return

        try:
            self._serial.write(b"Z\n")
            self._serial.flush()
            with self._lock:
                self._zero_requested_at = time.time()
        except Exception as e:
            print(f"UART TX error: {e}")

    def is_heading_settled(self, settle_time=0.15):
        """
        True once fresh telemetry has arrived *after* the last
        zero_imu() request, and enough time has passed for the
        Teensy to have actually applied the reset.
        Prevents CALIBRATE from trusting a stale pre-zero heading.
        """
        with self._lock:
            if self._zero_requested_at is None:
                return True
            fresh = self._telemetry['last_update'] > self._zero_requested_at
            elapsed = time.time() - self._zero_requested_at
            return fresh and elapsed > settle_time
    def get_telemetry(self):
        """Return latest telemetry dict."""
        with self._lock:
            return self._telemetry.copy()

    def is_alive(self, timeout_sec=1.0):
        """True if Teensy sent data within timeout_sec."""
        return (time.time() -
                self._telemetry['last_update']) < timeout_sec

    def get_message_count(self):
        return self._msg_count

    # ── Background receive loop ───────────────────────────────────────────

    def _receive_loop(self):
        while self._running:
            try:
                line = self._serial.readline()
                if not line:
                    continue
                text = line.decode('utf-8',
                                    errors='ignore').strip()
                if text:
                    self._parse_telemetry(text)
            except Exception as e:
                if self._running:
                    print(f"UART RX error: {e}")
                time.sleep(0.01)

    def _parse_telemetry(self, line: str):
        """
        Parse:
        T<rpm>,<ticks>,<heading>,<raw>,<sys>,<gyro>,<accel>,<mag>

        Example:
        T118.3,45231,91.2,181.5,3,3,3,3
        """
        if not line.startswith('T'):
            return
        try:
            parts = line[1:].split(',')

            if len(parts) != 8:
                return

            rpm = float(parts[0])
            ticks = int(parts[1])
            heading = float(parts[2])

            imu_raw = float(parts[3])

            imu_sys_cal = int(parts[4])
            imu_gyro_cal = int(parts[5])
            imu_accel_cal = int(parts[6])
            imu_mag_cal = int(parts[7])

            with self._lock:

                self._telemetry['actual_rpm'] = rpm
                self._telemetry['total_ticks'] = ticks
                self._telemetry['heading'] = heading

                self._telemetry['imu_raw'] = imu_raw

                self._telemetry['imu_sys_cal'] = imu_sys_cal
                self._telemetry['imu_gyro_cal'] = imu_gyro_cal
                self._telemetry['imu_accel_cal'] = imu_accel_cal
                self._telemetry['imu_mag_cal'] = imu_mag_cal

                self._telemetry['last_update'] = time.time()
            self._msg_count += 1

        except (ValueError, IndexError):
            pass  # malformed packet, ignore
