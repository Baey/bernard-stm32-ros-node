#ifndef _MICROROS_NODE_BOOL_HPP
#define _MICROROS_NODE_BOOL_HPP

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <bipedal_micro_ros/macros.hpp>

class FootContactNode
{
public:
    FootContactNode()
        : support(), node(), timer(), executor(), allocator(), publisherL(), publisherR(), msgL(), msgR() {}

    ~FootContactNode();

    static FootContactNode *instance;

    bool createEntities();

    void destroyEntities();

    void spin();

private:
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;
    rcl_allocator_t allocator;
    rcl_publisher_t publisherL;
    rcl_publisher_t publisherR;
    std_msgs__msg__Bool msgL;
    std_msgs__msg__Bool msgR;

    static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif