#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "opencv4/opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

float colour_error_perc = 50.0;
int dropped_frames = 0;
bool process_this_frame = true;
cv::Mat cv_image;
cv::Mat small_frame;
cv::Mat small_frame_hsv;
cv::Mat mask;
cv_bridge::CvImagePtr cv_ptr;
int height;
int width;
int last_turn = 0;

int finished_180 = 0;
int go_forward = 0;
//lower blue = 101 50 38
//upper blue = 110 255 255

void movement_orders(int image_dim_y, int image_dim_x, int cx, int cy, int mode,
double linear_vel_base = 0.5, double lineal_vel_min = 0.23, 
double angular_vel_base = 0.2, double movement_time = 0.02){

	const double delta_left_percentage_not_important = 0.05;
	const double FACTOR_LINEAR = 0.001;
    const double FACTOR_ANGULAR = 0.05;

	double origin[2] = {image_dim_x / 2.0, image_dim_y / 2.0};
	double centroid[2] = {(double)cx, (double)cy};
    double delta_left_right = centroid[0] - origin[0];
    /*if (fabs(delta_left_right) <= image_dim_x * delta_left_percentage_not_important) {

		//ROS_ERROR("DONMEYE GEREK YOK FARK AZ, delta_left_right = %f", delta_left_right);
			
		delta_left_right = 0.0;
    }*/
    double delta[2] = {delta_left_right, centroid[1]};

	if (mode == 1){
		cmd_vel.angular.z = angular_vel_base * delta[0] * FACTOR_ANGULAR * -1 * mode * 1.25;
		cmd_vel.linear.x = 0.60;
	}
	else{
		cmd_vel.angular.z = 1.5 * (delta_left_right/fabs(delta_left_right)) * -1 * mode;
		cmd_vel.linear.x = 0.0;
	}

	ROS_ERROR("angular: %f, angular_vel_base: %f, delta[0]: %f, factor_Angular: %f", cmd_vel.angular.z, angular_vel_base, delta[0], FACTOR_ANGULAR);
    //cmd_vel.linear.x = linear_vel_base - delta[1] * FACTOR_LINEAR; //hız ve açısal hız hesaplama todo
	/*if (mode < 0 ){
		if (cmd_vel.angular.z == 0) {
			cmd_vel.linear.x = -0.5;
		}
		else{
			cmd_vel.linear.x = 0.0;
		}
			
	}
	else{
		cmd_vel.linear.x = 0.5;
	}*/
	

	
	ROS_ERROR("LINEER: %f, ANGULAR: %f", cmd_vel.linear.x, cmd_vel.angular.z);
	cmd_vel_pub.publish(cmd_vel);
}



void moveRobot(int image_dim_y, int image_dim_x, int cx, int cy, std::vector<std::vector<cv::Point>> contours, int max_id,
               double linear_vel_base = 0.5, double lineal_vel_min = 0.23,
               double angular_vel_base = 0.2, double movement_time = 0.02)
{
    geometry_msgs::Twist cmd_vel;


	
	//ROS_ERROR("CX: %d, CY: %d, image_dim_y = %d, image_dim_x = %d", cx, cy, image_dim_y, image_dim_x);

	if (!contours.empty() && (cv::contourArea(contours[max_id]) > 33000 || contours.size() == 1 && cv::contourArea(contours[0]) > 5000)){
		//yolun sonundayız tek contour engel
		//ondan uzaklaş
		finished_180 = 0;
		cv::Moments moments = cv::moments(contours[max_id]);
		cv::Point target = cv::Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
		ROS_ERROR("ENGELE YAKLAŞTI ENGELDEN TERSE DÖNÜYOR");
		movement_orders(image_dim_y, image_dim_x, target.x, target.y, -1.5);
		ROS_ERROR("ENGELE YAKLAŞTI DÖNÜŞ BAŞLADI...");
		//ros::Duration(0.3).sleep();
		//cmd_vel_pub.publish(cmd_vel);
		ROS_ERROR("ENGELDEN KAÇINMA HAREKETİ BİTTİ");
		//cmd_vel_pub.publish(cmd_vel);
		
	}
	else if (fabs(cx) < image_dim_x && fabs(cy) < image_dim_y && cx != -1 && cy != -1) {
		finished_180 = 0;
		ROS_ERROR("ŞERİT TAKİP EDİLİYOR");
		movement_orders(image_dim_y, image_dim_x, cx, cy, 1);
		
	
    }
	else if (contours.empty() && !finished_180){
		//if (!go_forward){
			
			cmd_vel.linear.x = 0.65;
			cmd_vel.angular.z = 0.0;
			
			ROS_ERROR("CONTOUR YOK DUZ GIDIYOR 1SN BOYUNCA");
			ROS_ERROR("LINEER: %f, ANGULAR: %f", cmd_vel.linear.x, cmd_vel.angular.z);
			cmd_vel_pub.publish(cmd_vel);
			/*ros::Time init_time = ros::Time::now();
    		bool finished_movement_time = false;
    		ros::Rate rate_object(10);
			while (!finished_movement_time) {
        		ros::Duration delta = ros::Time::now() - init_time;
        		finished_movement_time = delta.toSec() >= 1.50;
        		rate_object.sleep();//1SN UYUYOR
    		}*/
			ros::Duration(1.5).sleep();
			go_forward = 1;
		//}
			
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 3.14;
		ROS_ERROR("1SN DUZ GITTI DONUYOR");
		ROS_ERROR("LINEER: %f, ANGULAR: %f", cmd_vel.linear.x, cmd_vel.angular.z);
		cmd_vel_pub.publish(cmd_vel);
		/*//ros::Time init_time = ros::Time::now();
    	//bool finished_movement_time = false;
		finished_movement_time = false;
    	//ros::Rate rate_object(10);
		while (!finished_movement_time) {
        	ros::Duration delta = ros::Time::now() - init_time;
        	finished_movement_time = delta.toSec() >= 1.00;
        	rate_object.sleep(1000);//1SN UYUYOR
    	}*/
		ros::Duration(1.85).sleep();
		ROS_ERROR("1SN DONDU");
		ROS_ERROR("LINEER: %f, ANGULAR: %f", cmd_vel.linear.x, cmd_vel.angular.z);
		finished_180 = 1;
	}

	if (cmd_vel.angular.z > 0) {
        last_turn = 1;
    } else if (cmd_vel.angular.z < 0) {
        last_turn = -1;
    } else if (cmd_vel.angular.z == 0) {
        last_turn = 0;
    }
	
}



void cameraCallBack(const sensor_msgs::Image::ConstPtr& camera){
	//ASAGIDA BULUNAN IF KOMUTU ORNEK OLARAK VERILMISTIR. SIZIN BURAYI DEGISTIRMENIZ BEKLENMEKTEDIR
	//BURDAN SONRASINI DEGISTIR

	process_this_frame = dropped_frames >= 2;
	if (process_this_frame){
		dropped_frames = 0;
		try{
			cv_ptr = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			cv_ptr = NULL;
		}

		if (cv_ptr != NULL){
			cv_image = cv_ptr->image;
			
			std::vector <int> lower_blue = {105, 50, 50};
			std::vector <int> upper_blue = {135, 255, 255};

			cv::resize(cv_image, small_frame, cv::Size(), 0.5, 0.5);
			cv::cvtColor(small_frame, small_frame_hsv, cv::COLOR_BGR2HSV);
			cv::inRange(small_frame_hsv, lower_blue, upper_blue, mask);

			cv::Mat res;
			cv::bitwise_and(small_frame, small_frame, res, mask);



			height = small_frame.rows;
			width = small_frame.cols;

			cv::Point target = cv::Point(-1, -1);
			std::vector<std::vector<cv::Point>> contours;
			cv::findContours(mask, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_TC89_L1);
			std::vector<cv::Point> centres;
			
			cv::drawContours(res, contours, -1, cv::Scalar(255,255,255), 1);
			double max_area = 0;
			double min_area = 1000000000;
			int max_id = 0;
			int min_id = 0;
			int current_x = 0;
			int current_y = 0;
			int current_target = 0;
			unsigned long int contour_cnt = contours.size();
			//TODO en büyük 2. contour'un merkezine git. aşagıdaki kod tüm contourlar arasında en büyük hariç İLK bulduğu contour'un merkezine gidiyor.
			ROS_ERROR("contour count: %lu", contour_cnt);
			for (int i = 0; i < contour_cnt; i++){
				int area = cv::contourArea(contours[i]);
				ROS_ERROR("current contour = %d, size = %d", i, area);
					
				cv::Moments moments = cv::moments(contours[i]);

				try{
					current_x = moments.m10 / moments.m00;
					current_y = moments.m01 / moments.m00;
					if (current_y > target.y){
						target = cv::Point(current_x, current_y);
						current_target = i;
					}
				}
				catch (const std::exception& e) {}

				if (area > max_area){
					max_area = area;
					max_id = i;
				}
				if (area < min_area && area > 1){
					min_area = area;
					min_id = i;
				}
			}

			ROS_ERROR("biggest contour: %d, smallest contour %d",max_id, min_id);
				
			if (current_x != 0 && current_y != 0){
				std::vector<std::vector<cv::Point>> max_contour;
				max_contour.push_back(contours[max_id]);
				cv::drawContours(res, max_contour, -1, cv::Scalar(0, 255, 0), 1);
				std::vector<std::vector<cv::Point>> target_contour;
				target_contour.push_back(contours[current_target]);
				cv::drawContours(res, target_contour, -1, cv::Scalar(0, 0, 255), 1);
				cv::circle(res, target, 10, cv::Scalar(0, 0, 255), -1);
				ROS_ERROR("CURRENT TARGET: %d", current_target);
			}
			
				
			//}
			

			cv::imshow("IMG", small_frame);
			cv::imshow("RES", res);
        	cv::waitKey(1);
			moveRobot(height, width, target.x, target.y, contours, max_id);
			cmd_vel_pub.publish(cmd_vel);
			ROS_ERROR("\n");
		}


	}
	else{
		dropped_frames += 1;
	}
	
	
	//BURDAN SONRASINA DOKUNMA

	//cmd_vel_pub.publish(cmd_vel);
}




int main(int argc, char **argv){
	ros::init(argc, argv, "man3_autonomy");
	ros::NodeHandle nh;

	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;

	ros::Subscriber camera_sub = nh.subscribe("/rtg/camera/rgb/image_raw", 1000, cameraCallBack);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/rtg/cmd_vel", 1000);

	ros::spin();

	return 0;
}