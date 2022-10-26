import rospy
import actionlib
from bin_picking.msg import PickObjectAction, PickObjectGoal
import cv2
from cv_bridge import CvBridge
import numpy as np

rospy.init_node("picking_node_tester")

client = actionlib.SimpleActionClient("pick_object", PickObjectAction)
print("waiting for server ready")
client.wait_for_server()
goal = PickObjectGoal()
#test_img = cv2.imread("testing_resources/background_test.png")
background_img = cv2.imread("testing_resources/background_test.png")
mask = cv2.imread("testing_resources/mask.bmp", cv2.IMREAD_GRAYSCALE)
depth_img = np.load("testing_resources/depth_img.npy")
bridge = CvBridge()


goal.command = "pick_object"
goal.mask = bridge.cv2_to_imgmsg(mask)
goal.reference_img = bridge.cv2_to_imgmsg(background_img)
goal.depth_img = bridge.cv2_to_imgmsg(depth_img)
print("sending request")
client.send_goal(goal)
client.wait_for_result()
result = client.get_result()
print(result)

goal.command = "place_object"
goal.position = [200, -250, 100]
print("sending request")
client.send_goal(goal)
client.wait_for_result()
result = client.get_result()
print(result)