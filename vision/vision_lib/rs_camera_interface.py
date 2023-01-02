import numpy as np
import rospy
import pyrealsense2 as rs


class RSCamera:
    def __init__(self, create_node=False):
        if create_node:
            rospy.init_node("test", anonymous=True)

        self.rs_pipeline = rs.pipeline()
        self.first_run = True

        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8, 30)
        profile = self.rs_pipeline.start(cfg)
        sensors = profile.get_device().query_sensors()

        rgb_cam = sensors[1]

        frames = None

        for i in range(90):
            frames = self.rs_pipeline.wait_for_frames()

    def __del__(self):
        self.rs_pipeline.stop()

    def get_image(self):
        frames = self.rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        color_image = np.asanyarray(color_frame.get_data())
        return color_image


if __name__ == "__main__":
    camera = RSCamera()
    camera.capture_image()