#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <iostream>
#include <ctime>

//해당 라이브러리는 opt/ros/noetic(melodic)/include 에서 확인 가능

ros::Publisher BOT_DATA_REQ_pub;
ros::Publisher ORD_REQ_pub;
ros::Subscriber BOT_DATA_RES_sub;
ros::Subscriber ORD_RES_sub;

std_msgs::UInt16MultiArray BOT_DATA_REQ_ARRY;
std_msgs::Float32MultiArray ORD_REQ_ARRY;

void BOT_DATA_RES_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
void ORD_RES_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);

int API_Number = 1, API1_CNT = 0, API2_CNT = 0;

//로봇으로부터 받는 데이터 초기화
int BOT_ID_RES = 0, BOT_STATE = 0, BOT_DELIVERY_STATE = 0, BOT_DELIVERY_PERC = 0, BOT_BATTERY_STATE = 0;
int BOT_CUR_SPEED = 0, BOT_Estimated_Arrive_Time = 0;
float BOT_CUR_LAT = 0, BOT_CUR_LNG = 0, BOT_DEST_LAT = 0, BOT_DEST_LNG = 0;

//로봇으로 보낼 데이터 선언
int BOT_ID_REQ = 1, BOT_DATA_TRIG = 1; 
int ORD_STORE_CD = 30499, ORD_NO = 1234567890, ORD_GATE_CODE = 1234, ORD_ELEV_CODE = 57832;
int ORD_ADDRESS_DETAIL = 103;
float ORD_DEST_LAT = 37.5077079, ORD_DEST_LNG = 127.0456153;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobinn_SERVER_node");
    // argc(argument count) = 메인 함수에 전달한 데이터의 갯수
    // argv(argument vector) = 데이터를 저장한 배열
    // mobinn_pub           = 노드의 이름

    ros::NodeHandle nh;

    BOT_DATA_REQ_pub = nh.advertise<std_msgs::UInt16MultiArray>("BOT_DATA_REQ", 1000);
    BOT_DATA_REQ_ARRY.data.resize(2);

    ORD_REQ_pub = nh.advertise<std_msgs::Float32MultiArray>("ORD_REQ", 1000);
    ORD_REQ_ARRY.data.resize(9);

    BOT_DATA_RES_sub = nh.subscribe<std_msgs::Float32MultiArray>("BOT_DATA_RES", 1000, BOT_DATA_RES_Callback);

    ORD_RES_sub = nh.subscribe<std_msgs::Float32MultiArray>("ORD_RES", 1000, ORD_RES_Callback);

    ros::Rate loop_rate(1);
    //반복 주기 : 1Hz

    ROS_INFO("Server Communication Node Activated");

    while (ros::ok())
    {
	ROS_INFO("ddd");
        if (API_Number == 1 && (API1_CNT < 2))
        {
	    ROS_INFO("ddd2");
            BOT_DATA_REQ_ARRY.data[0] = BOT_ID_REQ;
            BOT_DATA_REQ_ARRY.data[1] = BOT_DATA_TRIG;

            BOT_DATA_REQ_pub.publish(BOT_DATA_REQ_ARRY);
            ++API1_CNT;
            API2_CNT = 0;
        }

        else if (API_Number == 2 && API2_CNT < 2)
        {
            ORD_REQ_ARRY.data[0] = BOT_ID_REQ;
            ORD_REQ_ARRY.data[1] = ORD_STORE_CD;
            ORD_REQ_ARRY.data[2] = ORD_NO;
            ORD_REQ_ARRY.data[3] = ORD_GATE_CODE;
            ORD_REQ_ARRY.data[4] = ORD_ELEV_CODE;
            ORD_REQ_ARRY.data[5] = ORD_ADDRESS_DETAIL;
            ORD_REQ_ARRY.data[6] = ORD_DEST_LAT;
            ORD_REQ_ARRY.data[7] = ORD_DEST_LNG;

            ORD_REQ_pub.publish(ORD_REQ_ARRY);
            ++API2_CNT;
            API1_CNT = 0;
        }
        ros::spinOnce(); // loop_rate 주기를 지속적으로 받기 위해 필요
        loop_rate.sleep();
    }
    return 0;
}

void BOT_DATA_RES_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    BOT_ID_RES = msg->data[0];
    BOT_STATE = msg->data[1];
    BOT_DELIVERY_STATE = msg->data[2];
    BOT_DELIVERY_PERC = msg->data[3];
    BOT_BATTERY_STATE = msg->data[4];
    BOT_CUR_SPEED = msg->data[5];
    BOT_Estimated_Arrive_Time = msg->data[6];
    BOT_CUR_LAT = msg->data[7];
    BOT_CUR_LNG = msg->data[8];
}

void ORD_RES_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    BOT_ID_RES = msg->data[0];
    BOT_STATE = msg->data[1];
    BOT_DELIVERY_STATE = msg->data[2];
    BOT_DEST_LAT = msg->data[3];
    BOT_DEST_LNG = msg->data[4];
}
