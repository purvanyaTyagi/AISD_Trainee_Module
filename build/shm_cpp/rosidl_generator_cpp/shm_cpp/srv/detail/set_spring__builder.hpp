// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from shm_cpp:srv/SetSpring.idl
// generated code does not contain a copyright notice

#ifndef SHM_CPP__SRV__DETAIL__SET_SPRING__BUILDER_HPP_
#define SHM_CPP__SRV__DETAIL__SET_SPRING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "shm_cpp/srv/detail/set_spring__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace shm_cpp
{

namespace srv
{

namespace builder
{

class Init_SetSpring_Request_b
{
public:
  explicit Init_SetSpring_Request_b(::shm_cpp::srv::SetSpring_Request & msg)
  : msg_(msg)
  {}
  ::shm_cpp::srv::SetSpring_Request b(::shm_cpp::srv::SetSpring_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::shm_cpp::srv::SetSpring_Request msg_;
};

class Init_SetSpring_Request_k
{
public:
  Init_SetSpring_Request_k()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetSpring_Request_b k(::shm_cpp::srv::SetSpring_Request::_k_type arg)
  {
    msg_.k = std::move(arg);
    return Init_SetSpring_Request_b(msg_);
  }

private:
  ::shm_cpp::srv::SetSpring_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::shm_cpp::srv::SetSpring_Request>()
{
  return shm_cpp::srv::builder::Init_SetSpring_Request_k();
}

}  // namespace shm_cpp


namespace shm_cpp
{

namespace srv
{

namespace builder
{

class Init_SetSpring_Response_message
{
public:
  explicit Init_SetSpring_Response_message(::shm_cpp::srv::SetSpring_Response & msg)
  : msg_(msg)
  {}
  ::shm_cpp::srv::SetSpring_Response message(::shm_cpp::srv::SetSpring_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::shm_cpp::srv::SetSpring_Response msg_;
};

class Init_SetSpring_Response_success
{
public:
  Init_SetSpring_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetSpring_Response_message success(::shm_cpp::srv::SetSpring_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetSpring_Response_message(msg_);
  }

private:
  ::shm_cpp::srv::SetSpring_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::shm_cpp::srv::SetSpring_Response>()
{
  return shm_cpp::srv::builder::Init_SetSpring_Response_success();
}

}  // namespace shm_cpp

#endif  // SHM_CPP__SRV__DETAIL__SET_SPRING__BUILDER_HPP_
