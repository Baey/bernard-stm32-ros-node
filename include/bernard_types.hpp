#ifndef _BERNARD_TYPES_HPP
#define _BERNARD_TYPES_HPP

/// @brief Status types reported by MicroROS.
enum microROSStatus_t
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

/// @brief Structure representing the robot status.
struct BernardStatus
{
  bool IMUOnline;
  microROSStatus_t ROSStatus;
};

#endif // _BERNARD_TYPES_HPP