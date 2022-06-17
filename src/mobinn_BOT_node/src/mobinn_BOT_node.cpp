#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <iostream>
#include <ctime>

//해당 라이브러리는 opt/ros/noetic(melodic)/include 에서 확인 가능

ros::Subscriber BOT_DATA_REQ_sub;
ros::Subscriber ORD_REQ_sub;
ros::Publisher BOT_DATA_RES_pub;
ros::Publisher ORD_RES_pub;

std_msgs::Float32MultiArray BOT_DATA_RES_ARRY;
std_msgs::Float32MultiArray ORD_RES_ARRY;

void BOT_DATA_REQ_Callback(const std_msgs::UInt16MultiArray::ConstPtr &msg);
void ORD_REQ_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);

bool data_publish_TRIG = false, ord_res_publish_TRIG = false;

//서버로 보낼 데이터 선언
int BOT_ID = 1, BOT_STATE = 0, BOT_DELIVERY_STATE = 2, BOT_DELIVERY_PERC = 31;
int BOT_BATTERY_STATE = 73, BOT_CUR_SPEED = 3;
float BOT_CUR_LAT = 37.200640, BOT_CUR_LNG = 126.823383;
float BOT_DEST_LAT = 37.5077079, BOT_DEST_LNG = 127.0456153;
int Robot_value_test = 1;
int Estimated_Delivery_Time_s = 670;

//서버에서 받을 데이터 초기화
int BOT_ID_REQ = 0, BOT_DATA_TRIG = 0, ORD_STORE_CD = 0, ORD_NO = 0, ORD_GATE_CODE = 0, ORD_ELEV_CODE = 0;
int ORD_ADDRESS_DETAIL = 0;
float ORD_DEST_LAT = 0, ORD_DEST_LNG = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobinn_BOT_node_2");
    // argc(argument count) = 메인 함수에 전달한 데이터의 갯수
    // argv(argument vector) = 데이터를 저장한 배열
    // mobinn_pub           = 노드의 이름

    ros::NodeHandle nh;

    //로봇 데이터 RES publisher
    BOT_DATA_RES_pub = nh.advertise<std_msgs::Float32MultiArray>("BOT_DATA_RES", 1000);
    BOT_DATA_RES_ARRY.data.resize(9);

    //주문 명령 res publisher
    ORD_RES_pub = nh.advertise<std_msgs::Float32MultiArray>("ORD_RES", 1000);
    ORD_RES_ARRY.data.resize(5);

    //로봇 데이터 REQ subscriber
    BOT_DATA_REQ_sub = nh.subscribe<std_msgs::UInt16MultiArray>("BOT_DATA_REQ", 1000, BOT_DATA_REQ_Callback);

    //주문 명령 subscriber
    ORD_REQ_sub = nh.subscribe<std_msgs::Float32MultiArray>("ORD_REQ", 1000, ORD_REQ_Callback);

    ros::Rate loop_rate(1);
    //반복 주기 : 1Hz

    ROS_INFO("Bot Communication Node Activated");

    //도착 예상시간 계산 및 string 변환
    time_t now;
    time_t BOT_Estimated_Arrive_Time;
    time(&now);
    BOT_Estimated_Arrive_Time = now + Estimated_Delivery_Time_s;

    while (ros::ok())
    {
        if (data_publish_TRIG == true)
        {
            /************로봇 상태 데이터 Pub (int arry)******************/
            BOT_DATA_RES_ARRY.data[0] = BOT_ID;             // 로봇 ID
            BOT_DATA_RES_ARRY.data[1] = BOT_STATE;          // 로봇 현 상태
            BOT_DATA_RES_ARRY.data[2] = BOT_DELIVERY_STATE; // 배달 상태
            BOT_DATA_RES_ARRY.data[3] = BOT_DELIVERY_PERC;  // 배달 진행률 ex) 31%
            BOT_DATA_RES_ARRY.data[4] = BOT_BATTERY_STATE;  // 배터리 잔량 ex) 73%
            BOT_DATA_RES_ARRY.data[5] = BOT_CUR_SPEED;      // 로봇 속도 ex) 3kph
            BOT_DATA_RES_ARRY.data[6] = BOT_Estimated_Arrive_Time;
            BOT_DATA_RES_ARRY.data[7] = BOT_CUR_LAT;
            BOT_DATA_RES_ARRY.data[8] = BOT_CUR_LNG;

            BOT_DATA_RES_pub.publish(BOT_DATA_RES_ARRY);
        }
        else if (ord_res_publish_TRIG == true)
        {
            ORD_RES_ARRY.data[0] = BOT_ID;
            ORD_RES_ARRY.data[1] = BOT_STATE;
            ORD_RES_ARRY.data[2] = BOT_DELIVERY_STATE;
            ORD_RES_ARRY.data[3] = BOT_DEST_LAT;
            ORD_RES_ARRY.data[4] = BOT_DEST_LNG;
            ORD_RES_pub.publish(ORD_RES_ARRY);
            ord_res_publish_TRIG = false; //한번만 publish
        }
        ros::spinOnce(); // loop_rate 주기를 지속적으로 받기 위해 필요
        loop_rate.sleep();
    }

    return 0;
}

void BOT_DATA_REQ_Callback(const std_msgs::UInt16MultiArray::ConstPtr &msg)
{
    ROS_INFO("echo");
    BOT_ID_REQ = msg->data[0];
    BOT_DATA_TRIG = msg->data[1];

    if (BOT_DATA_TRIG == 1)
    {
        data_publish_TRIG = 1;
    }
    else
    {
        data_publish_TRIG = 0;
    }
}

void ORD_REQ_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    //주문 명령을 받으면, 명령 데이터를 robot 변수에 저장 후 publish TRIG 작동.
    //실제 로봇에서는 경로 계산이 완료되면 ord_res_publish_TRIG를 true로 변경하도록 개발 예정.

    BOT_ID_REQ = msg->data[0];
    ORD_STORE_CD = msg->data[1];
    ORD_NO = msg->data[2];
    ORD_GATE_CODE = msg->data[3];
    ORD_ELEV_CODE = msg->data[4];
    ORD_ADDRESS_DETAIL = msg->data[5];
    ORD_DEST_LAT = msg->data[6];
    ORD_DEST_LNG = msg->data[7];

    ord_res_publish_TRIG = true;
}
