import numpy as np
import rospy
from PIL import Image as pimg
import actionlib
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2


class RSCamera:
    def __init__(self, create_node=False):
        if create_node:
            rospy.init_node("test", anonymous=True)

        self.rs_pipeline = rs.pipeline()
        self.initiate_camera()

    #Destructor, stops pipeline
    def __del__(self):
        self.rs_pipeline.stop()


    def initiate_camera(self):
        cfg = rs.config()
        # cfg.enable_stream(realsense.stream.depth, 1280, 720, realsense.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8, 30)

        profile = self.rs_pipeline.start(cfg)
        sensors = profile.get_device().query_sensors()
        rgb_camera = sensors[1] ##make sure this is correct
        rgb_camera.set_option(rs.option.white_balance, 4600)
        rgb_camera.set_option(rs.option.exposure, 80)
        #rgb_camera.set_option(rs.option.saturation, 65)
        #rgb_camera.set_option(rs.option.contrast, 50)

        frames = None
        # wait for autoexposure to catch up
        for i in range(90):
            frames = self.rs_pipeline.wait_for_frames()

        self.first_run = False

    def get_image(self):
        if self.first_run:
            self.first_run_func()

        frames = self.rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        #Saves image
        #color_image_ready_to_save = pimg.fromarray(color_image, 'RGB')
        #color_image_ready_to_save.save(self.image_path)
        return np_img


if __name__ == "__main__":
    camera = RSCamera(True)
    img = camera.get_image()
    depth = camera.get_depth()
    depth = depth*120
    depth = depth.astype(np.uint8)
    cv2.imshow("d", depth)
    cv2.imshow("rgb", img)
    cv2.imwrite("background.png", img)
    #depth = camera.get_depth()
    #cv2.imshow("depth", depth)
    cv2.waitKey(0)
