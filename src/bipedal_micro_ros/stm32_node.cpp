#include <Arduino.h>
#include <bipedal_micro_ros/stm32_node.hpp>
#include <bipedal_micro_ros/macros.hpp>
#include <bipedal_wiring.hpp>

// Definition of the static member.
STM32Node *STM32Node::instance = nullptr;

/// @brief Destructor that destroys all created ROS entities.
STM32Node::~STM32Node()
{
    destroyEntities();
}

/// @brief Creates all necessary ROS entities.
/// @return true on success.
bool STM32Node::createEntities()
{
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));

    RCCHECK(rclc_node_init_default(&this->node, "stm32_publisher_rclc", "", &this->support));

    // Foot contact publishers
    RCCHECK(rclc_publisher_init_best_effort(
        &pubFootL,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
        "foot_contact_l"));
    
    RCCHECK(rclc_publisher_init_best_effort(
        &pubFootR,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
        "foot_contact_r"));
    
    // IMU publishers
    RCCHECK(rclc_publisher_init_best_effort(
        &pubIMU,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
        "quaternion"));

    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &this->timer,
        &this->support,
        RCL_MS_TO_NS(timer_timeout),
        this->timerCallback));

    STM32Node::instance = this;
    executor = rclc_executor_get_zero_initialized_executor();

    RCCHECK(rclc_executor_init(&this->executor, &this->support.context, 1, &this->allocator));

    RCCHECK(rclc_executor_add_timer(&this->executor, &this->timer));

    return true;
}

/// @brief Destroys all created ROS entities.
void STM32Node::destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rcl_publisher_fini(&pubFootL, &node);
    rcl_publisher_fini(&pubFootR, &node);
    rcl_publisher_fini(&pubIMU, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

/// @brief Spins the executor to process available callbacks.
void STM32Node::spin()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

/// @brief Timer callback that toggles the boolean messages and publishes them.
/// @param timer The timer object.
/// @param last_call_time The last call time.
void STM32Node::timerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (STM32Node::instance != nullptr)
    {
        STM32Node::instance->msgFootL.data = analogRead(ANALOG_PRESSURE_SENSOR);
        STM32Node::instance->msgFootR.data = STM32Node::instance->msgFootL.data + 10;
        STM32Node::instance->msgIMU.w = 0.0;
        STM32Node::instance->msgIMU.x = 0.0;
        STM32Node::instance->msgIMU.y = 0.0;
        STM32Node::instance->msgIMU.z = 0.0;
        rcl_publish(&STM32Node::instance->pubFootL, &STM32Node::instance->msgFootL, NULL);
        rcl_publish(&STM32Node::instance->pubFootR, &STM32Node::instance->msgFootR, NULL);
        rcl_publish(&STM32Node::instance->pubIMU, &STM32Node::instance->msgIMU, NULL);
    }
}