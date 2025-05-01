#include <Arduino.h>
#include <bernard_micro_ros/stm32_node.hpp>

// Definition of the static member.
STM32Node *STM32Node::instance = nullptr;

STM32Node::STM32Node(BernardSensors &sensors)
    : support(),
      node(),
      timer(),
      executor(),
      allocator(),
      pubFootL(),
      pubFootR(),
      pubQuat(),
      pubGyro(),
      pubAcc(),
      pubStatus(),
      msgFootL(),
      msgFootR(),
      msgQuat(),
      msgGyro(),
      msgAcc(),
      msgStatus(),
      sensors(&sensors)
{
}

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
        &pubQuat,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
        "quat"));
        
    RCCHECK(rclc_publisher_init_best_effort(
        &pubGyro,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "gyro"));

    RCCHECK(rclc_publisher_init_best_effort(
        &pubAcc,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Accel),
        "acc"));
    
    // Status publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &pubStatus,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "status"));

    const unsigned int timer_timeout = 1;
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
    rcl_publisher_fini(&pubQuat, &node);
    rcl_publisher_fini(&pubGyro, &node);
    rcl_publisher_fini(&pubAcc, &node);
    rcl_publisher_fini(&pubStatus, &node);
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
        std::vector<uint32_t> footPressure = STM32Node::instance->sensors->getFootPressure();
        STM32Node::instance->msgFootL.data = footPressure[0];
        STM32Node::instance->msgFootR.data = footPressure[1];

        imu::Quaternion quat = STM32Node::instance->sensors->getQuaternion();
        STM32Node::instance->msgQuat.w = quat.w();
        STM32Node::instance->msgQuat.x = quat.x();
        STM32Node::instance->msgQuat.y = quat.y();
        STM32Node::instance->msgQuat.z = quat.z();

        imu::Vector<3> linearAcc = STM32Node::instance->sensors->getLinearAcceleration();
        imu::Vector<3> angularAcc = STM32Node::instance->sensors->getAccelerometer();
        STM32Node::instance->msgAcc.linear.x = linearAcc.x();
        STM32Node::instance->msgAcc.linear.y = linearAcc.y();
        STM32Node::instance->msgAcc.linear.z = linearAcc.z();
        STM32Node::instance->msgAcc.angular.x = angularAcc.x();
        STM32Node::instance->msgAcc.angular.y = angularAcc.y();
        STM32Node::instance->msgAcc.angular.z = angularAcc.z();

        imu::Vector<3> gyro = STM32Node::instance->sensors->getGyroscope();
        STM32Node::instance->msgGyro.x = gyro.x();
        STM32Node::instance->msgGyro.y = gyro.y();
        STM32Node::instance->msgGyro.z = gyro.z();

        STM32Node::instance->msgStatus.data = 1;

        rcl_publish(&STM32Node::instance->pubFootL, &STM32Node::instance->msgFootL, NULL);
        rcl_publish(&STM32Node::instance->pubFootR, &STM32Node::instance->msgFootR, NULL);
        rcl_publish(&STM32Node::instance->pubQuat, &STM32Node::instance->msgQuat, NULL);
        rcl_publish(&STM32Node::instance->pubGyro, &STM32Node::instance->msgGyro, NULL);
        rcl_publish(&STM32Node::instance->pubAcc, &STM32Node::instance->msgAcc, NULL);
        rcl_publish(&STM32Node::instance->pubStatus, &STM32Node::instance->msgStatus, NULL);
    }
}