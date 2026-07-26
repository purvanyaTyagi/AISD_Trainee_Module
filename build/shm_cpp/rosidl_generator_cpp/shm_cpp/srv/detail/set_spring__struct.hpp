// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shm_cpp:srv/SetSpring.idl
// generated code does not contain a copyright notice

#ifndef SHM_CPP__SRV__DETAIL__SET_SPRING__STRUCT_HPP_
#define SHM_CPP__SRV__DETAIL__SET_SPRING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__shm_cpp__srv__SetSpring_Request __attribute__((deprecated))
#else
# define DEPRECATED__shm_cpp__srv__SetSpring_Request __declspec(deprecated)
#endif

namespace shm_cpp
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetSpring_Request_
{
  using Type = SetSpring_Request_<ContainerAllocator>;

  explicit SetSpring_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->k = 0.0;
      this->b = 0.0;
    }
  }

  explicit SetSpring_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->k = 0.0;
      this->b = 0.0;
    }
  }

  // field types and members
  using _k_type =
    double;
  _k_type k;
  using _b_type =
    double;
  _b_type b;

  // setters for named parameter idiom
  Type & set__k(
    const double & _arg)
  {
    this->k = _arg;
    return *this;
  }
  Type & set__b(
    const double & _arg)
  {
    this->b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shm_cpp::srv::SetSpring_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const shm_cpp::srv::SetSpring_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shm_cpp::srv::SetSpring_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shm_cpp::srv::SetSpring_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shm_cpp::srv::SetSpring_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shm_cpp::srv::SetSpring_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shm_cpp::srv::SetSpring_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shm_cpp::srv::SetSpring_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shm_cpp::srv::SetSpring_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shm_cpp::srv::SetSpring_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shm_cpp__srv__SetSpring_Request
    std::shared_ptr<shm_cpp::srv::SetSpring_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shm_cpp__srv__SetSpring_Request
    std::shared_ptr<shm_cpp::srv::SetSpring_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetSpring_Request_ & other) const
  {
    if (this->k != other.k) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetSpring_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetSpring_Request_

// alias to use template instance with default allocator
using SetSpring_Request =
  shm_cpp::srv::SetSpring_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace shm_cpp


#ifndef _WIN32
# define DEPRECATED__shm_cpp__srv__SetSpring_Response __attribute__((deprecated))
#else
# define DEPRECATED__shm_cpp__srv__SetSpring_Response __declspec(deprecated)
#endif

namespace shm_cpp
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetSpring_Response_
{
  using Type = SetSpring_Response_<ContainerAllocator>;

  explicit SetSpring_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetSpring_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shm_cpp::srv::SetSpring_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const shm_cpp::srv::SetSpring_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shm_cpp::srv::SetSpring_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shm_cpp::srv::SetSpring_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shm_cpp::srv::SetSpring_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shm_cpp::srv::SetSpring_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shm_cpp::srv::SetSpring_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shm_cpp::srv::SetSpring_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shm_cpp::srv::SetSpring_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shm_cpp::srv::SetSpring_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shm_cpp__srv__SetSpring_Response
    std::shared_ptr<shm_cpp::srv::SetSpring_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shm_cpp__srv__SetSpring_Response
    std::shared_ptr<shm_cpp::srv::SetSpring_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetSpring_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetSpring_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetSpring_Response_

// alias to use template instance with default allocator
using SetSpring_Response =
  shm_cpp::srv::SetSpring_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace shm_cpp

namespace shm_cpp
{

namespace srv
{

struct SetSpring
{
  using Request = shm_cpp::srv::SetSpring_Request;
  using Response = shm_cpp::srv::SetSpring_Response;
};

}  // namespace srv

}  // namespace shm_cpp

#endif  // SHM_CPP__SRV__DETAIL__SET_SPRING__STRUCT_HPP_
