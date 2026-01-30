// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_description:srv/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_DESCRIPTION__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_
#define MY_ROBOT_DESCRIPTION__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_description/srv/detail/set_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_description
{

namespace srv
{

namespace builder
{

class Init_SetVelocity_Request_angular_z
{
public:
  explicit Init_SetVelocity_Request_angular_z(::my_robot_description::srv::SetVelocity_Request & msg)
  : msg_(msg)
  {}
  ::my_robot_description::srv::SetVelocity_Request angular_z(::my_robot_description::srv::SetVelocity_Request::_angular_z_type arg)
  {
    msg_.angular_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_description::srv::SetVelocity_Request msg_;
};

class Init_SetVelocity_Request_linear_x
{
public:
  Init_SetVelocity_Request_linear_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_Request_angular_z linear_x(::my_robot_description::srv::SetVelocity_Request::_linear_x_type arg)
  {
    msg_.linear_x = std::move(arg);
    return Init_SetVelocity_Request_angular_z(msg_);
  }

private:
  ::my_robot_description::srv::SetVelocity_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_description::srv::SetVelocity_Request>()
{
  return my_robot_description::srv::builder::Init_SetVelocity_Request_linear_x();
}

}  // namespace my_robot_description


namespace my_robot_description
{

namespace srv
{

namespace builder
{

class Init_SetVelocity_Response_message
{
public:
  explicit Init_SetVelocity_Response_message(::my_robot_description::srv::SetVelocity_Response & msg)
  : msg_(msg)
  {}
  ::my_robot_description::srv::SetVelocity_Response message(::my_robot_description::srv::SetVelocity_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_description::srv::SetVelocity_Response msg_;
};

class Init_SetVelocity_Response_success
{
public:
  Init_SetVelocity_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_Response_message success(::my_robot_description::srv::SetVelocity_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetVelocity_Response_message(msg_);
  }

private:
  ::my_robot_description::srv::SetVelocity_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_description::srv::SetVelocity_Response>()
{
  return my_robot_description::srv::builder::Init_SetVelocity_Response_success();
}

}  // namespace my_robot_description

#endif  // MY_ROBOT_DESCRIPTION__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_
