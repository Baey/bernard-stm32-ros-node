#ifndef _STM32_NODE_HPP
#define _STM32_NODE_HPP

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int32.h>
#include <geometry_msgs/msg/quaternion.h>
#include <bipedal_micro_ros/macros.hpp>

class STM32Node
{
public:
    STM32Node()
        : support(), node(), timer(), executor(), allocator(), pubFootL(), pubFootR(), pubIMU(), msgFootL(), msgFootR(), msgIMU() {}

    ~STM32Node();

    static STM32Node *instance;

    bool createEntities();

    void destroyEntities();

    void spin();

private:
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;
    rcl_allocator_t allocator;
    rcl_publisher_t pubFootL;
    rcl_publisher_t pubFootR;
    rcl_publisher_t pubIMU;
    std_msgs__msg__UInt32 msgFootL;
    std_msgs__msg__UInt32 msgFootR;
    geometry_msgs__msg__Quaternion msgIMU;

    static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif // _STM32_NODE_HPP