3 Read from sensors
===================================

.. image:: images/sensor.png
  :width: 800
  :alt: Alternative text

todo=true

The code is presented here::

    #!/usr/bin/env python3

    import sys
    import rospy
    from fh_sensors_camera.srv import *
    import numpy as np
    import cv2
    import open3d as o3d
    from cv_bridge import CvBridge
    from cameraService.cameraClient import CameraClient
    from sensor_msgs.msg import LaserScan
    from locationService.client import LocationClient

    import datetime

    def laserscanCallback(msg):
        scan = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        print(scan)
        print(angle_min)
        print(angle_increment)

    if __name__ == "__main__":

        print("Starting")

        rospy.init_node("realsense_client_usage_example", anonymous=True)
        rate = rospy.Rate(5)

        #############################################################################################################################

        # Capture information from camera mounted on robot
        cam_robot = CameraClient(type = "camera_robot")

        # The camera only updates information when it is told to captureNewScene() otherwise all informaiton is static

        cam_robot.captureNewScene()

        img_rgb = cam_robot.getRGB()
        cv2.imshow("rgb image", img_rgb)
        cv2.waitKey(0)

        # Get the point cloud

        cloud, _ = cam_robot.getPointCloudStatic()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        o3d.visualization.draw_geometries([pcd])

        # Get a colored point cloud

        cam_robot.getUvStatic()
        cloud, rgb = cam_robot.getPointCloudStatic()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        pcd.colors = o3d.utility.Vector3dVector(rgb)
        o3d.visualization.draw_geometries([pcd])

        cv2.destroyAllWindows()

        ##############################################################################################################################

        # Lidar scanner

        # Get the estimated position of the receiver
        locClient = LocationClient()
        location = locClient.getLocation()
        print(location)

        rospy.sleep(3)


        # get raw sensor readings through a topic
        rospy.Subscriber("/sensors/lidar/scan", LaserScan, laserscanCallback)
        i = 0

        while i < 5:
            rate.sleep()
            i += 1
        