#!/usr/bin/env python
from __future__ import print_function
from lib2to3.pytree import Node
import sys
import math
from tokenize import Double
import numpy as np
import time

from  numpy import array, dot
from quadprog import solve_qp
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GapBarrier:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read the algorithm parameter paramters form params.yaml
        
        lidarscan_topic =rospy.get_param('~scan_topic')
        odom_topic = rospy.get_param('~odom_topic')
        drive_topic = rospy.get_param('~nav_drive_topic')
        camera_topic = rospy.get_param('~camera_topic')

        self.scan_beams=rospy.get_param('~scan_beams')
        self.max_steering_angle=rospy.get_param('~max_steering_angle')
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.wheelbase=rospy.get_param('~wheelbase')
        self.k_p=rospy.get_param('~k_p')
        self.k_d=rospy.get_param('~k_d')
        self.car_velocity=rospy.get_param('~car_velocity')
        self.stop_distance=rospy.get_param('~stop_distance')
        self.distance_decay=rospy.get_param('~distance_decay')
        self.viewing_angle=rospy.get_param('~viewing_angle')

        self.angle_bl = 4.71239 #270
        self.angle_br = 1.57079 # 90
        self.angle_al = 3.49066 #20
        self.angle_ar = 2.79253 # 160
        self.safe_distance = 0.4 #m, try changing to 2 if this doesn't work
        self.cvbridge = CvBridge()
        # Add your subscribers for LiDAR scan and Odomotery here
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.camera_callback, queue_size=10)
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.intrensic_callback)
        
        # Add your publisher for Drive topic here
        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)      
        
        # Initialize varables as needed
        self.right_beam_angle = 1.91986 #110
        self.left_beam_angle = 4.36332  #250
        self.str_index = int(round((self.scan_beams*self.right_beam_angle)/(2*math.pi)))
        self.end_index = int(round((self.scan_beams*self.left_beam_angle)/(2*math.pi)))
        self.angle_inc = (2*math.pi)/self.scan_beams
        self.fov = (self.end_index-self.str_index+1)*self.angle_inc
        self.angle_cen = self.fov/2
        
        self.object_right = 20
        self.object_left = 20
        self.wl0 = np.array([[0,0],[-1,0]])
        self.wr0= np.array([[0,0],[1,0]])
        self.alpha = 0
        self.intrensic = []
        self.vel = 0
        self.cam_obst = []

        self.counter=0

    # Optional function to pre-process LiDAR by considering only those LiDAR returns within a FOV in front of the vehicle;    

    def preprocess_lidar(self, ranges):                  
        data_1 = []
        for i in range(self.end_index-self.str_index+1):
            if ranges[self.str_index+i] <= self.safe_distance:
                data_1.append([0,i*self.angle_inc-self.angle_cen])
            elif ranges[self.str_index+i] <= self.max_lidar_range:
                data_1.append([ranges[self.str_index+i], i*self.angle_inc-self.angle_cen])
            else:
                data_1.append([self.max_lidar_range, i*self.angle_inc-self.angle_cen])
        return np.array(data_1)
    

    # Optional function to find the the maximum gap in front the vehicle 
    def find_max_gap(self, proc_ranges):
        current_sum = 0
        current_start = 0
        max_sum = 0
        max_start = 0
        max_end = 0
        
        for i in range(self.end_index-self.str_index+1):
            if proc_ranges[i,0] != 0:
                if current_start == 0:
                    current_start = i
                    current_sum = 0
                current_sum += proc_ranges[i,0]
                if current_sum > max_sum :
                    max_sum = current_sum
                    max_start = current_start
                    max_end = i
            else:
                current_start = 0
                current_sum = 0
        
        return max_start, max_end
                
    #Optional function to find the best direction of travel
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # Returns index of best (furthest point) in ranges
    def find_best_point(self, start_i, end_i, proc_ranges):
        range_sum = 0
        best_heading = 0
        for i in range(start_i, end_i+1):
            range_sum += proc_ranges[i, 0]
            best_heading += proc_ranges[i, 0]*proc_ranges[i, 1]
            
        if range_sum != 0:
            best_heading = best_heading/range_sum
            
        return best_heading
        
    
    # Optional function to set up and solve the optimization problem for parallel virtual barriers 
    def getWalls(self, left_obstacles, right_obstacles, wl0, wr0, c_mat):
        a_mat = np.zeros((3))
        g_mat = np.array(([1.0, 0, 0], [0, 1.0, 0], [0, 0, 0.0001]))
        
        bl = np.full((self.object_left), 1.0, dtype=np.float64)
        br = np.full((self.object_right), 1.0, dtype=np.float64)
        b = np.concatenate((br, bl, np.array([-0.99, -0.99])))

        Cl = left_obstacles
        Cr = right_obstacles
        C1 = np.vstack((Cr, br)) 
        C2 = np.vstack((-Cl, -bl))  
        C = np.hstack((C1, C2))    
        C = np.hstack((C, np.array([[0, 0], [0, 0], [1.0, -1.0]])))
            
        w = solve_qp(g_mat.astype(np.float64),a_mat.astype(np.float64),C.astype(np.float64),b.astype(np.float64),0)[0]
        wl = np.array([w[0]/(w[2] + 1),w[1]/(w[2] + 1)])
        wr = np.array([w[0]/(w[2] - 1),w[1]/(w[2] - 1)])
        
        return wl, wr

    # This function is called whenever a new set of LiDAR data is received; bulk of your controller implementation should go here 
    def lidar_callback(self, data):      

        # Pre-process LiDAR data as necessary
        proc_ranges = self.preprocess_lidar(data.ranges)
        for cam_obst in self.cam_obst:
            proc_ranges[cam_obst] = 0
        
        # Find the widest gap in front of vehicle
        start_i, end_i = self.find_max_gap(proc_ranges)

        # Find the Best Direction of Travel
        heading_angle = self.find_best_point(start_i, end_i, proc_ranges)
        
        # Set up the QP for finding the two parallel barrier lines
        index_l = int(round((self.angle_bl-self.angle_al)/(data.angle_increment*self.object_left)))
        index_r = int(round((self.angle_ar-self.angle_br)/(data.angle_increment*self.object_right)))
        
        mod_angle_al = (self.angle_al + heading_angle)%(2*math.pi)
            
        mod_angle_br = (self.angle_br + heading_angle)%(2*math.pi)
        
        start_index_l = int(round(mod_angle_al/data.angle_increment))
        start_index_r = int(round(mod_angle_br/data.angle_increment))
        
        obs_pts_l = np.zeros((2,self.object_left))
        obs_pts_r = np.zeros((2,self.object_right))
        
        for a in range(0,self.object_left):
            obj_index = (start_index_l+a*index_l) % self.scan_beams
            obj_range = data.ranges[obj_index]
            if obj_range >= self.max_lidar_range:
                obj_range = self.max_lidar_range
            
            obs_pts_l[0][a] = obj_range*math.cos(mod_angle_al+a*index_l*data.angle_increment-math.pi)
            obs_pts_l[1][a] = obj_range*math.sin(mod_angle_al+a*index_l*data.angle_increment-math.pi)
            
        for b in range(0,self.object_right):
            obj_index = (start_index_r+b*index_r) % self.scan_beams
            obj_range = data.ranges[obj_index]
            if obj_range >= self.max_lidar_range:
                obj_range = self.max_lidar_range
        
            obs_pts_r[0][b] = obj_range*math.cos(mod_angle_br+b*index_r*data.angle_increment-math.pi)
            obs_pts_r[1][b] = obj_range*math.sin(mod_angle_br+b*index_r*data.angle_increment-math.pi)
            
            
        # Solve the QP problem to find the barrier lines parameters w,b
        wl , wr = self.getWalls(obs_pts_l, obs_pts_r, self.wl0, self.wr0, data)
        self.wl0 = wl
        self.wr0 = wr
        
        # Compute the values of the variables needed for the implementation of feedback linearizing+PD controller
        dis_l = 1/(math.sqrt(np.dot(wl.T,wl)))
        dis_r = 1/(math.sqrt(np.dot(wr.T,wr)))
        
        wl_hat = dis_l*wl
        wr_hat = dis_r*wr
        
        # Compute the steering angle command
        if self.vel >= 0.01 or self.vel <= -0.01:
            dlr_tilda = dis_l -dis_r
            dlr_dot = self.vel*(wl_hat[0]-wr_hat[0])
            delta = math.atan((self.wheelbase*(self.k_p*dlr_tilda + self.k_d*dlr_dot))/((self.vel**2)*(-wl_hat[1]+wr_hat[1])))
        else:
            delta = 0 
        
        if delta >= self.max_steering_angle:
            delta = self.max_steering_angle
        elif delta <= -self.max_steering_angle:
            delta = -self.max_steering_angle
        
        # Find the closest obstacle point in a narrow field of view in fronnt of the vehicle and compute the velocity command accordingly    
        closest_dis = min(data.ranges[-int((self.viewing_angle/data.angle_increment))+int(self.scan_beams/2):int((self.viewing_angle/data.angle_increment))+int(self.scan_beams/2)])
        des_vel = self.car_velocity*(1-math.exp(-max(closest_dis-self.stop_distance,0)/self.distance_decay))
        
        # Publish the steering and speed commands to the drive topic
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.header.frame_id = "base_link"           
        self.drive_msg.drive.steering_angle = delta
        self.drive_msg.drive.speed = des_vel
        
        self.drive_pub.publish(self.drive_msg)

    # Odometry callback 
    def odom_callback(self, odom_msg):
        # update current speed
         self.vel = odom_msg.twist.twist.linear.x
    
    def camera_callback(self, depth):
        if self.counter<8:
            self.counter+=1
        else:
            pass
            try:
                if self.intrensic:
                    self.cv_image_data = depth
                else:
                    self.cv_image_data = None
            except Exception as e:
                #print(e)
                return
            
            self.cam_obst = []

            cv_image = self.cvbridge.imgmsg_to_cv2(self.cv_image_data, self.cv_image_data.encoding)
            #for i in range(np.size(cv_image)/30000):
            #print(cv_image[0])
            
            camera_coords_x=np.zeros((cv_image.shape[0]/32,cv_image.shape[1]/16))
            camera_coords_y=np.zeros((cv_image.shape[0]/32,cv_image.shape[1]/16))
            camera_coords_z=np.zeros((cv_image.shape[0]/32,cv_image.shape[1]/16))
            
            #we need our focal length f, cx, cy
            cx=self.intrensic[2]
            cy=self.intrensic[5]
            f=self.intrensic[0]

            for j in range(cv_image.shape[0]/2,cv_image.shape[0],16):
                for k in range(0,cv_image.shape[1],16):
                    camera_coords_x[j/32][k/16]=(-1*(k-cx)*cv_image[j][k]/f)/1000 #this is our y coordinate in baselink
                    camera_coords_y[j/32][k/16]=(-1*(j-cy)*cv_image[j][k]/f-86)/1000 #this is our z coordinate in baselink
                    camera_coords_z[j/32][k/16]=(cv_image[j][k]+287)/1000 #this is our x coordinate in our baselink

            for j in range(0,camera_coords_x.shape[0]):
                for k in range(0,camera_coords_x.shape[1]):
                    if j<((camera_coords_x.shape[0]/2)-1):
                        if camera_coords_z[j][k]<self.safe_distance and k>0.27*camera_coords_x.shape[1] and k<0.73*camera_coords_x.shape[1]:
                            lidar_scan_index=int(round(((k/(camera_coords_x.shape[1]))-0.27)*174))
                            self.cam_obst.append(lidar_scan_index)
            self.counter=0
        


 
    def intrensic_callback(self, info):
        self.intrensic = info.K
        #print(self.intrensic)


def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)