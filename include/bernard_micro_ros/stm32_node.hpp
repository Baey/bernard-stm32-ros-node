#ifndef _STM32_NODE_HPP
#define _STM32_NODE_HPP

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/accel.h>
#include <geometry_msgs/msg/vector3.h>
#include <bernard_micro_ros/macros.hpp>
#include <bernard_sensors.hpp>

class STM32Node
{
public:
    STM32Node(BernardSensors &sensors);
    
    ~STM32Node();

    static STM32Node *instance;

    bool createEntities();

    void destroyEntities();

    void spin();

private:
    BernardSensors *sensors;
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;
    rcl_allocator_t allocator;
    rcl_publisher_t pubFootL;
    rcl_publisher_t pubFootR;
    rcl_publisher_t pubQuat;
    rcl_publisher_t pubGyro;
    rcl_publisher_t pubAcc;
    rcl_publisher_t pubStatus;
    std_msgs__msg__UInt32 msgFootL;
    std_msgs__msg__UInt32 msgFootR;
    geometry_msgs__msg__Quaternion msgQuat;
    geometry_msgs__msg__Vector3 msgGyro;
    geometry_msgs__msg__Accel msgAcc;
    std_msgs__msg__UInt8 msgStatus;

    static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif // _STM32_NODE_HPP