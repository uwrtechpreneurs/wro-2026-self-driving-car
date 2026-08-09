# lidar_reader.py
import subprocess
import threading
import numpy as np
import re

class LidarC1:
    """
    Reads RPLIDAR C1 data via official Slamtec SDK binary.
    Works reliably regardless of Python library compatibility.
    """
    
    ULTRA_SIMPLE_PATH = "/home/techpreneurs/FEC/rplidar_sdk/output/Linux/Release/ultra_simple"
    LIDAR_PORT = "/dev/ttyUSB0"
    BAUD = "460800"
    
    def __init__(self):
        self._angles = np.array([])
        self._distances = np.array([])
        self._qualities = np.array([])
        self._lock = threading.Lock()
        self._scan_count = 0
        self._running = False
        
        # Buffer to accumulate one full scan
        self._buffer_angles = []
        self._buffer_distances = []
        self._buffer_qualities = []
        self._last_angle = 0
        
        # Regex to parse: "theta: 349.83 Dist: 02073.00 Q: 47"
        self._pattern = re.compile(
            r'theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)\s+Q:\s*(\d+)'
        )
    
    def start(self):
        self._running = True
        self._process = subprocess.Popen(
            [self.ULTRA_SIMPLE_PATH,
             "--channel", "--serial",
             self.LIDAR_PORT, self.BAUD],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1  # line buffered
        )
        
        self._thread = threading.Thread(
            target=self._read_loop,
            daemon=True
        )
        self._thread.start()
        
        # Wait for first complete scan
        print("Waiting for first scan...")
        import time
        while self._scan_count == 0:
            time.sleep(0.05)
        print("LiDAR ready.")
    
    def _read_loop(self):
        for line in self._process.stdout:
            if not self._running:
                break
            
            match = self._pattern.search(line)
            if not match:
                continue
            
            angle = float(match.group(1))
            distance = float(match.group(2))
            quality = int(match.group(3))
            
            # Detect scan boundary: angle wraps from ~359° back to ~0°
            if angle < self._last_angle - 180:
                # New scan started — save completed scan
                if len(self._buffer_angles) > 100:
                    with self._lock:
                        self._angles = np.array(
                            self._buffer_angles, dtype=np.float32)
                        self._distances = np.array(
                            self._buffer_distances, dtype=np.float32)
                        self._qualities = np.array(
                            self._buffer_qualities, dtype=np.float32)
                        self._scan_count += 1
                
                # Start fresh buffer
                self._buffer_angles = []
                self._buffer_distances = []
                self._buffer_qualities = []
            
            self._buffer_angles.append(angle)
            self._buffer_distances.append(distance)
            self._buffer_qualities.append(quality)
            self._last_angle = angle
    
    def get_scan(self):
        """
        Returns latest complete scan as numpy arrays.
        Filtered: removes blind zone and low quality.
        """
        with self._lock:
            a = self._angles.copy()
            d = self._distances.copy()
            q = self._qualities.copy()
        
        if len(a) == 0:
            return np.array([]), np.array([])
        
        # Filter out blind zone and bad readings
        valid = (
            (d >= 50) &        # C1 blind zone = 50mm
            (d <= 3500) &      # max useful range
            (q > 10)           # minimum quality
        )
        
        return a[valid], d[valid]
    
    def get_scan_count(self):
        return self._scan_count
    
    def stop(self):

        self._running = False

        if hasattr(self, "_process"):
            self._process.terminate()

        if hasattr(self, "_thread"):
            self._thread.join(timeout=1)
