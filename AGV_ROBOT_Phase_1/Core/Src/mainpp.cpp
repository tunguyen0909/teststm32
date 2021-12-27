/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include "mainpp.h"
#include <ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
//#include "kal_man.h"

#define LENGTH_BETWEEN_2_WHEEL 0.376 	// m unit
#define WHEEL_RADIUS 0.0475				// m unit

double g_vel_angular_left = 0;			//r/s unit
double g_vel_angular_right = 0;			//r/s unit


uint8_t pData;
char dataTX[62];
float num[10];
float a;
int i =0;
int j= 0;
//#include "main.h"
extern UART_HandleTypeDef huart3;
//SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
extern int vanToc1;
char button1[] = "button_1";
char button2[] = "button_2";
char button3[] = "button_3";
double vxRE,vyRE,wzRE;
int countTick = HAL_GetTick();

//void led0_cb(const std_msgs::Float32& msg);
void messageCb( const geometry_msgs::Twist& cmd_msg);
void convert_vel_linear_and_angular_to_vel_linear_2_wheel(double Vx, double Wz);
void convert_vel_linear_2_wheel_to_vel_linear_and_angular(double g_vel_angular_right, double g_vel_angular_left);

ros::NodeHandle nh;

//Publisher Magneticfield
//sensor_msgs::MagneticField magneticField;
//Publisher IMU
sensor_msgs::Imu imu;


//Publisher Position command
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
//IMU
//ros::Publisher pub_magnetic("imu/mag",&magneticField);
ros::Publisher pub_imu("imu/data", &imu);

//Subscriber and Publisher Velocity
geometry_msgs::Twist raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel",&raw_vel_msg);
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
  if(huart->Instance == USART3)
  	{
  		if(pData != 'A')
  		{
  			if(pData != 'B')
  			{
  				if((pData >= 46 && pData <= 57) || pData == 32 || pData == 45)
  				{
  					dataTX[i] = pData;
  					i++;
  				}
  			}
  		}

  		if(pData == 'B')
  		{
  			i = 0;
  			char *ptr;
  			ptr = strtok(dataTX," ");
  			while(ptr != NULL)
  			{
  				num[j] = atof(ptr);
  				ptr = strtok(NULL," ");
  				j++;
  				if(j == 10)
  				{
  					j = 0;
  					break;
  				}
  			}
  		}
  	}
  	HAL_UART_Receive_IT(&huart3, &pData, 1);
}

void setup(void)
{
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.advertise(raw_vel_pub);
  nh.advertise(chatter);
  nh.advertise(pub_imu);
}

void loop(void)
{

	if(W1 > 0 && W2 > 0 && W3 > 0 && W4 > 0)
	{
		tien();
	}
	else if(W1 < 0 && W2 < 0 && W3 < 0 && W4 < 0)
	{
		lui();
	}
	else if(W1 < 0 && W2 > 0 && W3 < 0 && W4 > 0)
	{
		xoayTrai();
	}
	else if(W1 > 0 && W2 < 0 && W3 > 0 && W4 < 0)
	{
		xoayPhai();
	}
	else if(W1 > 0 && W2 < 0 && W3 < 0 && W4 > 0)
	{
		ngangPhai();
	}
	else if(W1 < 0 && W2 > 0 && W3 > 0 && W4 < 0)
	{
		ngangTrai();
	}
	else
	{
		dung();
	}

	if(HAL_GetTick() - countTick > 100)
	{
		//publish velocity
		convert_vel_linear_2_wheel_to_vel_linear_and_angular(w1, w2);
		raw_vel_msg.linear.x = vx;
		raw_vel_msg.angular.z = wz;
		raw_vel_pub.publish(&raw_vel_msg);

		//publish imu/data
		imu.header.frame_id = "imu_link";
		imu.header.stamp = nh.now();
		imu.orientation.w = num[0];
		imu.orientation.x = num[1];
		imu.orientation.y = num[2];
		imu.orientation.z = num[3];
		imu.angular_velocity.x = num[7];
		imu.angular_velocity.y = num[8];
		imu.angular_velocity.z = num[9];
		imu.linear_acceleration.x = num[4];
		imu.linear_acceleration.y = num[5];
		imu.linear_acceleration.z = num[6];
		pub_imu.publish(&imu);
		countTick = HAL_GetTick();
	}

	nh.spinOnce();
}

void messageCb( const geometry_msgs::Twist& cmd_msg)
{
	vxRE = cmd_msg.linear.x; //rad/s
	wzRE = cmd_msg.angular.z; //rad/s
	convert_vel_linear_and_angular_to_vel_linear_2_wheel(vxRE,wzRE);
}

void convert_vel_linear_and_angular_to_vel_linear_2_wheel(double Vx, double Wz)
{
	g_vel_angular_right = (((2*Vx + Wz*LENGTH_BETWEEN_2_WHEEL)/(2*WHEEL_RADIUS))/WHEEL_RADIUS)/haiPI;  //vòng/giây
	g_vel_angular_left = (((2*Vx - Wz*LENGTH_BETWEEN_2_WHEEL)/(2*WHEEL_RADIUS))/WHEEL_RADIUS)/haiPI;   //vòng/giây
}

void convert_vel_linear_2_wheel_to_vel_linear_and_angular(double g_vel_angular_right, double g_vel_angular_left)
{
	vx = (WHEEL_RADIUS/2)*(g_vel_angular_right + g_vel_angular_left)*haiPI*WHEEL_RADIUS; //rad/s
	wz = (WHEEL_RADIUS/2)*(g_vel_angular_right - g_vel_angular_left)*haiPI*WHEEL_RADIUS; //rad/s
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_10)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0)
		{
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			  str_msg.data = button1;
			  chatter.publish(&str_msg);
		}
	}

	else if(GPIO_Pin == GPIO_PIN_11)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0)
		{
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			  str_msg.data = button2;
			  chatter.publish(&str_msg);
		}
	}

	else if(GPIO_Pin == GPIO_PIN_12)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0)
		{
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			  str_msg.data = button3;
			  chatter.publish(&str_msg);
		}
	}
}


