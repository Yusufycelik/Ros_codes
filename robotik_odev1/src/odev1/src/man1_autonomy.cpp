#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#define MAX_ANGULAR 6.0
#define MIN_ANGULAR 1.0
#define MAX_LINEAR 1.6
#define MIN_LINEAR 0.6
#define LINEAR_INC 0.05
#define LINEAR_DEC 0.1
#define EPS1 0.05


ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

int state = 0;
float speed = 0;
float turning_speed = 0;
float min_dist_to_wall = 0;

float calcAvg(const sensor_msgs::LaserScan::ConstPtr& laser,int start,int end){
    int i;
    float sum = 0.0;
    int range = end-start;
    for(i=start;i<=end;i++){
        sum += laser->ranges[i];
    }
    return sum/(float)range;
}

float calcMin(const sensor_msgs::LaserScan::ConstPtr& laser,int start,int end){
    int i;
	float min = laser->ranges[start];	
    for(i=start + 1;i<=end;i++){
        if (min > laser->ranges[i]){
			min = laser->ranges[i];
		}
    }
    return min;
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
	//ASAGIDA BULUNAN IF KOMUTU ORNEK OLARAK VERILMISTIR. SIZIN BURAYI DEGISTIRMENIZ BEKLENMEKTEDIR
	//BURDAN SONRASINI DEGISTIR
	float right_0_45_avg = calcAvg(laser,180,360); // 0 - 45
	float left_0_45_avg = calcAvg(laser,720,900) ;
	float center_n15_15_avg = calcAvg(laser,480,600);
	float center_n5_5_avg = calcAvg(laser, 520, 560);
	float right_0_30_avg = calcAvg(laser,180,300); // 0 - 30
	float left_0_30_avg = calcAvg(laser,780,900);

	float right_n15_15_min = calcMin(laser, 120, 240); //-15 - 15
	float left_n15_15_min  = calcMin(laser, 840, 960);

	float side_n15_15_min_diff = right_n15_15_min - left_n15_15_min;
	float side_0_45_avg_diff =  right_0_45_avg-left_0_45_avg;
	


	if (state == 1){ //sola dön
		
		if (side_0_45_avg_diff > 0.02 && right_0_30_avg > 0.4){
			//dönme bitti
			cmd_vel.angular.z = 0.0;
			state = 0;
		}
	}
	else if (state == 2){ // sağa dön
		//durma koşulu
		if (side_0_45_avg_diff < 0.02 && left_0_30_avg > 0.4){
			//dönme bitti;
			cmd_vel.angular.z = 0.0;
			state = 0;
		}
	}
	else if (state == 3){ //180 dön
		//durma koşulu
		if(center_n15_15_avg>0.75){
            state = 0;
        }
        else{

			cmd_vel.linear.x = 0;
            cmd_vel.angular.z = MAX_ANGULAR * 1.5;
			
        }
	}
	else if (state == 4){//çok hafif geri gel
		if (center_n5_5_avg > 0.6){
			state = 0;
		}
		else{
			cmd_vel.linear.x = -0.2;
			cmd_vel.angular.z = 0.0;
		}
	}

	if (state < 3){
		//merkezi hız ve dönüş hesabı:
		if (state == 1){
			//sola dönerken hız hesabı sol kısmımıza olan mesafeye göre belirlenicek
			min_dist_to_wall = calcMin(laser, 180,360);
			turning_speed = fabs(side_0_45_avg_diff) * 10;
			if (turning_speed < MIN_ANGULAR) turning_speed = MIN_ANGULAR;
			else if (turning_speed > MAX_ANGULAR) turning_speed = MAX_ANGULAR;
			if (left_n15_15_min < 0.35) turning_speed = 0.0;
		}
		else if (state == 2){
			//SAGA DÖNERKEN İSE hız hesabı sağ tarafıma olan mesafeye göre belirlenicek
			min_dist_to_wall = calcMin(laser, 720, 900);
			turning_speed = fabs(side_0_45_avg_diff) * -10;
			if (turning_speed > -MIN_ANGULAR) turning_speed = -MIN_ANGULAR;
			else if (turning_speed < -MAX_ANGULAR) turning_speed = -MAX_ANGULAR;
			if (right_n15_15_min < 0.35) turning_speed = 0.0;
		}
		else if (state == 0){
			//düz giderken önümüzdeki mesafeye göre, dönüş hızımız 0
			min_dist_to_wall = calcMin(laser, 510, 570);
			turning_speed = 0;
		}
		speed = min_dist_to_wall * 2.5;
		if (speed < MIN_LINEAR) speed = MIN_LINEAR;
		if (speed > MAX_LINEAR) speed = MAX_LINEAR;
		if (min_dist_to_wall < 0.15) cmd_vel.linear.x = 0;
		else if (speed < cmd_vel.linear.x) cmd_vel.linear.x -= LINEAR_INC;
		else if (speed > cmd_vel.linear.x) cmd_vel.linear.x += LINEAR_DEC;

		cmd_vel.angular.z = turning_speed;

		if (center_n5_5_avg < 0.2){//duvar dibindeyse geri gel
			state = 4;
		}
		else if(center_n15_15_avg < 0.45 && fabs(side_0_45_avg_diff) < EPS1 * 2){
			state = 3;//180 dön
		}
		else if((side_0_45_avg_diff <= -EPS1 && center_n15_15_avg >= 0.35) || (right_n15_15_min <0.3) && (fabs(side_n15_15_min_diff) > EPS1)){
			state = 1; // SOLA DONUS
		}
		else if((side_0_45_avg_diff >= EPS1 && center_n15_15_avg >= 0.35) || (left_n15_15_min <0.3) && (fabs(side_n15_15_min_diff) > EPS1)){
			state = 2; // SAGA DONUS
		}
	}
	
	//BURDAN SONRASINA DOKUNMA

	cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "man1_autonomy");
	ros::NodeHandle nh;

	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	ros::Subscriber laser_sub = nh.subscribe("/rtg/hokuyo", 1000, laserCallBack);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/rtg/cmd_vel", 1000);

	ros::spin();
	return 0;
}


void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){

}




int main(int argc, char **argv){
	ros::init(argc, argv, "man1_autonomy");
	ros::NodeHandle nh;

	ros::Subscriber laser_sub = nh.subscribe("/rtg/hokuyo", 1000, laserCallBack);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/rtg/cmd_vel", 1000);

	ros::spin();
	return 0;
}