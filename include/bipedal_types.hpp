#ifndef _BIPEDAL_TYPES_HPP
#define _BIPEDAL_TYPES_HPP

/// @brief Status types reported by MicroROS.
enum MicroROSStatus
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

/// @brief Structure representing the robot status.
struct RobotStatus
{
  bool IMUOnline;
  MicroROSStatus ROSStatus;
};

#endif // _BIPEDAL_TYPES_HPP