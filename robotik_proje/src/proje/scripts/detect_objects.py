#!/usr/bin/env python3
import cv2
import sys
import time
import rospy
import tf
import numpy as np
from tf import transformations as tf_trans
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
from qreader import QReader
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from math import cos, sin, radians ,degrees, sqrt
import math
from scipy import ndimage
import multiprocessing
#80 derece tüm yatay ekran
#800 piksel var
#80/800 = her piksel 0.1 derece
#((x1+x2)/2)*0.1 ekranın en solundan yaptıgı açıyı verir
#lidar genişligi 270 derece, (270-80)/2 = 95 sol ve sağdan
#lidarın en soluna yaptıgı açı = 95 + int((x1+x2)/2)*0.1)
#ranges'daki değeri ranges[(95 + int((x1+x2)/2)*0.1)) * 4]
#+-12'ye bakılarak ortalama bir değer alınabilir
# 

rospy.init_node('qr_code_reader')
listener = tf.TransformListener()
object_pub = rospy.Publisher('/detected_objects', String, queue_size=10)
marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
image_pub = rospy.Publisher("sensor_msgs/Image2", Image, queue_size=10)
marker_array = MarkerArray()
bridge = CvBridge()
qreader = QReader()

templates = ['inhalation_hazard','non-flammable_gas','oxygen','poison','radioactive','dangerous_when_wet','combustible','organic_peroxide','corrosive', 'explosive', 'flammable_gas','flammable_solid']
x = 0
y = 0
x1 = 400
x2 = 400
y1 = 400
y2 = 400
ranges_arr = []
switch = False
draw_counter = 0
hazard_text_draw_counter = 0
hazard_text_draw_switch = True
yaw_degrees = 0
bounding_box_qr = []
decoded_text = ""
cv_image = None
qr_marker_list = []
cylinder_marker_list = []
process_this_frame = True
dropped_frames = 0
hazard_text = ""
img_middle = None
hazmat_marker_list = []
hazmat_preds = []


def calc_distance_between_2_points(a, b):
    """"2 nokta arası öklid mesafesini bulur"""
    distance = sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    return distance

def check_if_near(arr, point, max_dist=1.5, min_dist_between_marks = 1.0):
    """noktanın verilen arr listesindeki herhangi bir noktaya yakın olup olmadığına \n
        ve noktanın arabaya çok uzak olup olmadıgına bakar"""
    global x
    global y
    if calc_distance_between_2_points((x,y), point) > max_dist:
        return True
    for p in arr:
        if calc_distance_between_2_points(p, point) < min_dist_between_marks:
            return True
    return False

def get_deltas(distance, angle):
    #nesne açısı = araba açısı - (40-angle)
    obj_angle = yaw_degrees - (angle - 40)
    x_d = distance * cos(radians(obj_angle))
    y_d = distance * sin(radians(obj_angle))
    return x_d, y_d, obj_angle

def get_distance_to_camera_from_angle(angle, range=4):
    """nesnenin araca olan mesafesini verir\n
        lidar sensörü ekranın en solundan 95 derece daha solda başladıgı\n
        için +95 ekliyoruz"""
    angle_local = int(angle) + 95
    i = angle_local * 4
    avg = calc_avg(i - range, i + range)
    return avg

def calc_angle_from_pixel(x):
    """pixelin ekranın en soluna yaptıgı açıyı derece cinsinden verir"""
    angle_to_left = x * 0.1
    return angle_to_left

def calc_avg(start, end):
    """belirli aralıktaki lidar uzaklıklarının ortalamasını verir"""
    sum = 0
    r = end - start
    for i in range(start, end):
        sum2 = 0
        for j in range(len(ranges_arr)):
            sum2 += ranges_arr[j][i]
        sum2 = sum2 / len(ranges_arr)
        sum+=sum2
    return sum / r

def laser_callback(laser):
    global ranges_arr
    if len(ranges_arr) == 5:
        ranges_arr.pop(0)
    ranges_arr.append(laser.ranges)

def search_qr(img):
    global x
    global y
    global switch
    global bounding_box_qr
    global decoded_text
    global draw_counter
    global qr_marker_list
    img  = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)    

    qr_codes = qreader.detect_and_decode(image=img, return_bboxes=True)

    if switch == True:
        if draw_counter < 5:
            cv2.rectangle(cv_image, bounding_box_qr[0], bounding_box_qr[1], (255,0,0), 3)
            cv2.putText(cv_image, decoded_text, (bounding_box_qr[0][0]-15, bounding_box_qr[0][1]-15), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
            draw_counter = draw_counter+1  
        else:
            switch = False
            draw_counter = 0
    for qr_code in qr_codes:
        x1,y1,x2,y2 = qr_code[0]
        text = qr_code[1]
        if text != None:
            switch = True
            bounding_box_qr = [(x1,y1), (x2, y2)]
            decoded_text = text
            cv2.rectangle(cv_image, (x1,y1), (x2, y2), (255,0,0), 3)
            angle = calc_angle_from_pixel(int((x1+x2)/2))
            distance = get_distance_to_camera_from_angle(angle)
            xA, yA, obj_angle = get_deltas(distance, angle)
            message = String()
            message.data = "QR Code Detected: " + text
            object_pub.publish(message)
            #print("car positions: x={}, y={}, angle={} | mark positions: x={}, y={}, angle={}".format(x, y, yaw_degrees, x+xA, y+yA, obj_angle))
            #qr kod için beyaz kutu koy
            #eğer zaten işaretlenmişse işaretleme
            #5,5
            #5.5, 4.5
            if check_if_near(qr_marker_list, (x+xA, y+yA), 2) == False:
                put_marker(x+xA, y+yA, 1, 1, 1, 1)
                qr_marker_list.append((x+xA, y+yA))

def search_blue_cylinder(img):
    global x
    global y
    global cylinder_marker_list
    lower_blue = np.array([105, 50, 50], dtype=np.uint8)
    upper_blue = np.array([135, 255, 255], dtype=np.uint8)

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a mask based on the blue color range
    mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

    # Apply the mask to the image
    res = cv2.bitwise_and(img, img, mask=mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the biggest contour
    max_area = 0
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is not None and max_area > 80000:
        # Draw the biggest contour on the image
        cv2.drawContours(cv_image, [max_contour], -1, (255, 255, 255), 3)
        

        # Find the center of the biggest contour
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            max_contour_x = int(M["m10"] / M["m00"])
            max_contour_y = int(M["m01"] / M["m00"])
            cv2.putText(cv_image, "CYLINDER", (max_contour_x, max_contour_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)

        angle = calc_angle_from_pixel(max_contour_x)
        
        distance = get_distance_to_camera_from_angle(angle, 24)
        xA, yA, obj_angle = get_deltas(distance, angle)
        #print(angle, "|", yaw_degrees, "|", obj_angle)
        #print("car positions: x={}, y={}, angle={} | mark positions: x={}, y={}, angle={}".format(x, y, yaw_degrees, x+xA, y+yA, obj_angle))
        #qr kod için beyaz kutu koy
        #eğer zaten işaretlenmişse işaretleme
        #5,5
        #5.5, 4.5
        message = String()
        message.data = "Blue Cylinder Detected"
        object_pub.publish(message)
        if check_if_near(cylinder_marker_list, (x+xA, y+yA), 2, 0.25) == False and max_contour_x > 250 and max_contour_x < 550:
            put_marker(x+xA, y+yA, 3, 0, 0, 1)
            cylinder_marker_list.append((x+xA, y+yA))

def toSIFT(image1,image2):
    """image1 ve image2 arasında sift algoritması ile template matching yapar"""
    sift = cv2.SIFT_create()
    keypoints1, descriptors1 = sift.detectAndCompute(image1, None)
    keypoints2, descriptors2 = sift.detectAndCompute(image2, None)

    # FLANN matcher
    flann = cv2.FlannBasedMatcher()
    matches = flann.knnMatch(descriptors1, descriptors2, k=2)

    # Custom matcher with ratio test
    good_matches = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)

    # Calculate similarity ratio based on good matches
    try:
        similarity_ratio = len(good_matches) / min(len(keypoints1), len(keypoints2))
    except:
        
        #print("zero division")
        return 0
    return similarity_ratio

def match_template(template):
    img2 = cv2.imread("/home/deno/homeworks_ws/src/proje/scripts/hazard_logos/" + template + ".png")
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    img2 = ndimage.rotate(img2,-45)

    img_middle2 = img_middle.copy()
    gray_img_middle2 = cv2.cvtColor(img_middle2, cv2.COLOR_BGR2GRAY)
    gray_img_middle2 = np.clip(gray_img_middle2 * 2.5, 0, 255).astype(np.uint8)

    return toSIFT(gray_img_middle2, img2)

def search_hazmat():
    #gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    #brightness_factor = 9.5  # Adjust this value to control the brightness level
    #gray = np.clip(gray * brightness_factor, 0, 255).astype(np.uint8)
    #gray = gray[200:-200,200:-200]
    global hazmat_preds
    pool = multiprocessing.Pool()
    simRatio = pool.map(match_template, templates)
    pool.close()
    pool.join()

    #index = simRatio.index(max(simRatio))
    index = np.argmax(simRatio)
    hazmat_text = templates[index]
    threshold = 0.2
    #if hazmat_text == 'flammablegas':
    #    threshold = 0.7

    print(simRatio[index], "|", hazmat_text)
    if simRatio[index] > threshold:
        if len(hazmat_preds) == 3:
            hazmat_preds.pop(0)
        hazmat_preds.append(hazmat_text)
        result = all(element == hazmat_preds[0] for element in hazmat_preds)
        print(result)
        print(hazmat_preds)
        if result and len(hazmat_preds) == 3:
            return hazmat_text
        #print("\n", hazmat_text, "|", simRatio[index], "\n")
        
    return ""

def thresh_callback(img):
    """ekranda hazard tespit edildiginde çagırılıyor\n
    ekrandaki en büyük contour'u bulup onu çizer
    üstüne de text değişkenindeki yazıyı yazar"""
    global hazard_text_draw_switch
    global hazard_text_draw_counter
    global hazard_text
    global img_middle
    threshold = 100
    image = img.copy()
    image = image[300:-300, 300:-300]
    img_middle = image
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    brightness_factor = 2.5  # Adjust this value to control the brightness level
    image = np.clip(image * brightness_factor, 0, 255).astype(np.uint8)
    canny_output = cv2.Canny(img_gray, threshold, threshold * 2)
    kernel = np.ones((5,5), np.uint8)
    canny_output = cv2.dilate(canny_output, kernel, iterations=2)

    contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_poly = [None] * len(contours)
    boundRect = [None] * len(contours)

    largest_contour_index = -1
    largest_contour_area = -1

    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])

        contour_area = cv2.contourArea(c)
        if contour_area > largest_contour_area:
            largest_contour_area = contour_area
            largest_contour_index = i
    
    x1 = 400
    y1 = 400
    x2 = 400
    y2 = 400
    if largest_contour_index >= 0:
        x1 = int(boundRect[largest_contour_index][0]) + 300
        x2 = int(boundRect[largest_contour_index][0] + boundRect[largest_contour_index][2]) + 300
        y1 = int(boundRect[largest_contour_index][1]) + 300
        y2 = int(boundRect[largest_contour_index][1] + boundRect[largest_contour_index][3]) + 300
        area = (x2-x1)*(y2-y1)
                
        print(area)
        if area > 400:
            hazard_text = search_hazmat()
            return x1, y1, x2, y2
    return 10000, 10000, 10000, 10000

def draw_hazard_to_img(x1, y1, x2, y2):
    if len(hazard_text) > 0:
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        #radius = int(max((x2 - x1) // 1.5, (y2 - y1) // 1.5))
        radius = 100
        cv2.circle(cv_image, (center_x, center_y), radius, (0, 0, 255), 2)
        cv2.putText(cv_image, hazard_text, (x1, y1-15), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
        message = String()
        message.data = "Hazard Image Detected: " + hazard_text
        object_pub.publish(message)
        
def image_callback(msg):
    global cv_image
    global process_this_frame
    global dropped_frames
    global hazmat_marker_list
    global x1
    global x2
    global y1
    global y2
    global x
    global y
    process_this_frame = dropped_frames >= 5
    
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        #resimdeki qr code'lar, silindirler ve hazmatlar aranacak
        search_qr(cv_image)
        search_blue_cylinder(cv_image)
        if process_this_frame:
            dropped_frames = 0
            x1, y1, x2, y2 = thresh_callback(cv_image)
        else:
            dropped_frames+=1
        if len(hazard_text) > 0:
            angle = calc_angle_from_pixel(int((x1+x2)/2))
            distance = get_distance_to_camera_from_angle(angle)
            if distance < 0.75:
                draw_hazard_to_img(x1, y1, x2, y2)
            xA, yA, obj_angle = get_deltas(distance, angle)
            if check_if_near(hazmat_marker_list, (x+xA, y + yA), 0.75, 0.5) == False:
                
                put_marker(x+xA, y+yA, 2, 1, 0, 0)
                hazmat_marker_list.append((x+xA, y+yA))
            
    except Exception as e:
        pass
        #print("Error processing image: ", e)

def put_marker(x, y, marker_type, r, g, b):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.lifetime = rospy.Duration()
    marker_array.markers.append(marker)
    id = 0
    for m in marker_array.markers:
        m.id = id
        id = id + 1

    marker_pub.publish(marker_array)
    
def coordinate_listener():
    global x
    global y
    global yaw_degrees
    try:
        listener.waitForTransform('/map', 'rtg/base_link', rospy.Time(), rospy.Duration(1.0))
    except:
        pass
    
    #rate = rospy.Rate(5.0)
    #while not rospy.is_shutdown():
    try:
        trans, rot = listener.lookupTransform('/map', 'rtg/base_link', rospy.Time(0))
        x = trans[0] #araba x
        y = trans[1] #araba y
        euler = tf_trans.euler_from_quaternion(rot)

        yaw = euler[2]

        if yaw < 0:
            yaw += 2 * math.pi

        yaw_degrees = degrees(yaw) # bu arabanın haritaya göre yaptıgı açı, bu, araba konumu, kamera-nesne arasındaki açı ve nesnenin kameraya olan uzaklıgı ile (hepsi elimizde var) market koordinatı hesaplanabilir

    except:
        pass
        #rate.sleep()
    
if __name__ == '__main__':
    qr_pub = rospy.Publisher('/qrcode', String, queue_size=10)
    rospy.Subscriber('/rtg/camera/rgb/image_raw', Image, image_callback)
    rospy.Subscriber('rtg/hokuyo', LaserScan, laser_callback)
    
    
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            coordinate_listener()
            #print(cv_image)
            ros_image = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            #print(ros_image)
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = "sensor_msgs/Image2"
            image_pub.publish(ros_image)
        except:
            pass
            #print("hata")

        rate.sleep()
    
    rospy.spin()