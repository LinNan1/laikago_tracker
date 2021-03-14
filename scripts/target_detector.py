#!/usr/bin/python
import rospy,message_filters,cv2
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge
import dlib
import sys
import thread
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, TransformStamped
import tf2_ros,tf_conversions
class TargetDetector:
    def __init__(self):
        self.image_res_pub = rospy.Publisher('/laikago_traker/image_res',Image,queue_size=10)
        self.depth_res_pub = rospy.Publisher('/laikago_traker/depth_res',Image,queue_size=10)
        self.target_uv_pub = rospy.Publisher('/laikago_traker/target_uv',Point,queue_size=10)
        self.target_distance_pub = rospy.Publisher('/laikago_traker/target_distance',Float32,queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.camera_intrinsics = np.array([
            [607.48, 0.0, 319.197 ], 
            [0.0, 607.655,237.708 ], 
            [0.0, 0.0, 1.0]])
        self.t = TransformStamped()


        self.image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw',Image)
        self.bridge = CvBridge()
        self.time_syn = message_filters.ApproximateTimeSynchronizer([self.image_sub,self.depth_sub], 10, 0.1, allow_headerless=True)
        self.time_syn.registerCallback(self.sub_callback)

        self.play_buffer = []
        self.selected =  False
        self.initBB = None

    def select_target_thread(self):
        window_name = 'Press "s" to select'
        cv2.namedWindow(window_name,cv2.WINDOW_AUTOSIZE)
        while True:
            if len(self.play_buffer) > 0:
                image = self.play_buffer.pop(0)
                cv2.imshow(window_name,image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('s') or key == 27:
                cv2.setWindowTitle(window_name,'Select a ROI and then press SPACE or ENTER button!')
                self.initBB = cv2.selectROI(window_name,image,fromCenter=False,showCrosshair=False)
                break
        cv2.destroyWindow(window_name)
        # init
        self.algorithm_init(image)
        self.selected = True

    def algorithm_init(self,image):
        x,y,w,h = self.initBB

        # aruco

        # correlation
        # self.object_tracker = dlib.correlation_tracker()
        # self.object_tracker.start_track(image,dlib.rectangle(x,y,x+w,y+h))
 
        # meanshift
        # xmin,ymin,xmax,ymax = x,y,x+w,y+h
        # self.track_wnd = (x,y,w,h)
        # roi = image[ymin:ymax,xmin:xmax] # region of interest
        # hsv_roi = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv_roi, (0,60,32), (180,255,255))
        # self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0,180])
        # cv2.normalize(self.roi_hist, self.roi_hist,0,255,cv2.NORM_MINMAX)
        # self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 1)

        # CamShift
        # xmin,ymin,xmax,ymax = x,y,x+w,y+h
        # self.track_wnd = (x,y,w,h)
        # roi = image[ymin:ymax,xmin:xmax] # region of interest
        # hsv_roi = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv_roi, (0,60,32), (180,255,255))
        # self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0,180])
        # cv2.normalize(self.roi_hist, self.roi_hist,0,255,cv2.NORM_MINMAX)
        # self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 1)

        # opencv tracker
        # self.object_tracker = cv2.TrackerKCF_create()
        # self.object_tracker = cv2.TrackerBoosting_create()
        # self.object_tracker = cv2.TrackerMIL_create()
        # self.object_tracker = cv2.TrackerTLD_create()
        # self.object_tracker = cv2.TrackerMedianFlow_create()
    
        # self.object_tracker.init(image,self.initBB)

    def algorithm_run(self,image):
        x,y,w,h = -1,-1,-1,-1

        # aruco
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        if len(corners) > 0:
            ids = ids.flatten()
            for markerCorner,markerID in zip(corners,ids):
                if markerID == 11:
                    markerCorner = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = markerCorner
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    cv2.line(image, topLeft, topRight, (255, 255, 255), 2)
                    cv2.line(image, topRight, bottomRight, (255, 255, 255), 2)
                    cv2.line(image, bottomRight, bottomLeft, (255, 255, 255), 2)
                    cv2.line(image, bottomLeft, topLeft, (255, 255, 255), 2)
                    cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                    x,y = topLeft[0],topLeft[1]
                    w,h = topRight[0] - topLeft[0], bottomLeft[1] - topLeft[1] 
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                    break

        # correlation
        # self.object_tracker.update(image)
        # target_rect = self.object_tracker.get_position()
        # pt1 = (int(target_rect.left()), int(target_rect.top()))
        # pt2 = (int(target_rect.right()), int(target_rect.bottom()))
        # x,y = pt1[0],pt1[1]
        # w,h = pt2[0] - pt1[0],pt2[1] - pt1[1]
        # cv2.rectangle(image, pt1, pt2, (255, 255, 255), 2)

        # meanshift
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1)
        # retval, self.track_wnd = cv2.meanShift(dst,self.track_wnd,self.term_crit)
        # x,y,w,h = self.track_wnd
        # cX, cY = x+w/2, y+h/2
        # cX = 640-1 if cX >= 640 else cX
        # cY = 480-1 if cY >= 480 else cY
        # obj_center = (cX,cY)
        # cv2.rectangle(image, (x,y), (x+w,y+h), (255, 255, 255), 2)

        # CamShift
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1)
        # retval, self.track_wnd = cv2.CamShift(dst,self.track_wnd,self.term_crit)
        # x,y,w,h = self.track_wnd
        # cX, cY = x+w/2, y+h/2
        # cX = 640-1 if cX >= 640 else cX
        # cY = 480-1 if cY >= 480 else cY
        # obj_center = (cX,cY)
        # cv2.rectangle(image, (x,y), (x+w,y+h), (255, 255, 255), 2)

        # opencv tracker
        # (success, box) = self.object_tracker.update(image)
        # if success:
        #     (x,y,w,h) = [int(v) for v in box]
        #     cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,255),2)

        # resize for remote play
        image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        image = cv2.resize(image, (320,240), interpolation = cv2.INTER_AREA)

        return image, (x, y, w, h)

    def sub_callback(self,image,depth):
        # convert
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
        # target center
        cX,cY = (-1,-1)
        distance = -1
        if self.selected or True:
        # if self.selected:
            cv_image, (x,y,w,h) = self.algorithm_run(cv_image)
            if x >= 0:
                cX,cY = x+w/2,y+h/2
                cX = 640-1 if cX >= 640 else cX
                cY = 480-1 if cY >= 480 else cY
        else:
            # replay to select target
            if len(self.play_buffer) > 5:
                self.play_buffer.pop(0)
            self.play_buffer.append(cv_image)  # select img

        pt = Point()
        pt.x = cX
        pt.y = cY
        pt.z = 1.0
        self.target_uv_pub.publish(pt)

        
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = "camera"
        self.t.child_frame_id = "target"


        if cX != -1:
            depth_array = np.array(depth_image, dtype=np.float32)
            distance = depth_array[cY][cX]/1000 + 0.000000001
            P_uv = np.array([[cX],[cY],[1.0]])
            P_xyz = distance*np.dot(np.linalg.inv(self.camera_intrinsics),P_uv)
            # rospy.loginfo('%f, %f, %f',P_xyz[0][0],P_xyz[1][0],P_xyz[2][0])
            
            theta = 0.0
            self.t.transform.translation.x = P_xyz[0][0]
            self.t.transform.translation.y = P_xyz[1][0]
            self.t.transform.translation.z = P_xyz[2][0]
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
            self.t.transform.rotation.x = q[0]
            self.t.transform.rotation.y = q[1]
            self.t.transform.rotation.z = q[2]
            self.t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(self.t)
        else:
            self.t.transform.rotation.w = 1.0
        
        self.target_distance_pub.publish(distance)

        self.image_res_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "passthrough"))

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        self.depth_res_pub.publish(self.bridge.cv2_to_imgmsg(depth_colormap, "passthrough"))

        

def main(args):
    rospy.init_node('target_detector', anonymous=False)
    target_detector = TargetDetector()
    # thread.start_new_thread(target_detector.select_target_thread,())
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
   
if __name__ == '__main__':
    main(sys.argv)