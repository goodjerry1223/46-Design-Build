import numpy as np
try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser

    class SimpleLaser(Laser):
        def __init__(self):
            super().__init__(scan_size=360, scan_rate_hz=5,
                             detection_angle_degrees=360, distance_no_detection_mm=4000, offset_mm=0)

    class SLAMSystem:
        def __init__(self, map_size_pixels=100, map_size_meters=10):
            self.slam = RMHC_SLAM(SimpleLaser(), map_size_pixels, map_size_meters)
            self.slam.hole_width_mm = 2000
            self.mapbytes = bytearray(map_size_pixels * map_size_pixels)
            self.x = 0
            self.y = 0
            self.theta = 0
            self.map_size_pixels = map_size_pixels
        def update(self, scan):
            self.slam.update(scan)
            self.x, self.y, self.theta = self.slam.getpos()
            self.slam.getmap(self.mapbytes)
        def get_pose(self):
            return self.x / 1000.0, self.y / 1000.0, np.radians(self.theta)
        def get_map(self):
            return np.reshape(np.frombuffer(self.mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))
except ImportError:
    # 无breezyslam则降级为虚拟SLAM
    class SLAMSystem:
        def __init__(self, map_size_pixels=100, map_size_meters=10):
            self.x, self.y, self.theta = 1, 0, 0
            self.map_size_pixels = map_size_pixels
            self.map = np.zeros((map_size_pixels, map_size_pixels), dtype=np.uint8)
        def update(self, scan): pass
        def get_pose(self): return self.x, self.y, self.theta
        def get_map(self): return self.map
