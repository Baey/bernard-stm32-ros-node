#include <bipedal_micro_ros/foot_contact_node.hpp>
#include <bipedal_micro_ros/macros.hpp>

// Definition of the static member.
FootContactNode *FootContactNode::instance = nullptr;

/// @brief Destructor that destroys all created ROS entities.
FootContactNode::~FootContactNode()
{
    destroyEntities();
}

/// @brief Creates all necessary ROS entities.
/// @return true on success.
bool FootContactNode::createEntities()
{
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));

    RCCHECK(rclc_node_init_default(&this->node, "bool_publisher_rclc", "", &this->support));

    RCCHECK(rclc_publisher_init_best_effort(
        &publisherL,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "foot_contact_l"));
    
    RCCHECK(rclc_publisher_init_best_effort(
        &publisherR,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "foot_contact_r"));

    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &this->timer,
        &this->support,
        RCL_MS_TO_NS(timer_timeout),
        this->timerCallback));

    FootContactNode::instance = this;
    executor = rclc_executor_get_zero_initialized_executor();

    RCCHECK(rclc_executor_init(&this->executor, &this->support.context, 1, &this->allocator));

    RCCHECK(rclc_executor_add_timer(&this->executor, &this->timer));

    return true;
}

/// @brief Destroys all created ROS entities.
void FootContactNode::destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rcl_publisher_fini(&publisherL, &node);
    rcl_publisher_fini(&publisherR, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

/// @brief Spins the executor to process available callbacks.
void FootContactNode::spin()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

/// @brief Timer callback that toggles the boolean messages and publishes them.
/// @param timer The timer object.
/// @param last_call_time The last call time.
void FootContactNode::timerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (FootContactNode::instance != nullptr)
    {
        rcl_publish(&FootContactNode::instance->publisherL, &FootContactNode::instance->msgL, NULL);
        rcl_publish(&FootContactNode::instance->publisherR, &FootContactNode::instance->msgR, NULL);
        FootContactNode::instance->msgL.data = !FootContactNode::instance->msgL.data;
        FootContactNode::instance->msgR.data = !FootContactNode::instance->msgL.data;
    }
}