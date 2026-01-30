// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from shm_cpp:srv/SetSpring.idl
// generated code does not contain a copyright notice

#ifndef SHM_CPP__SRV__DETAIL__SET_SPRING__TRAITS_HPP_
#define SHM_CPP__SRV__DETAIL__SET_SPRING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "shm_cpp/srv/detail/set_spring__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace shm_cpp
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetSpring_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: k
  {
    out << "k: ";
    rosidl_generator_traits::value_to_yaml(msg.k, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetSpring_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: k
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "k: ";
    rosidl_generator_traits::value_to_yaml(msg.k, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetSpring_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace shm_cpp

namespace rosidl_generator_traits
{

[[deprecated("use shm_cpp::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const shm_cpp::srv::SetSpring_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  shm_cpp::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shm_cpp::srv::to_yaml() instead")]]
inline std::string to_yaml(const shm_cpp::srv::SetSpring_Request & msg)
{
  return shm_cpp::srv::to_yaml(msg);
}

template<>
inline const char * data_type<shm_cpp::srv::SetSpring_Request>()
{
  return "shm_cpp::srv::SetSpring_Request";
}

template<>
inline const char * name<shm_cpp::srv::SetSpring_Request>()
{
  return "shm_cpp/srv/SetSpring_Request";
}

template<>
struct has_fixed_size<shm_cpp::srv::SetSpring_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<shm_cpp::srv::SetSpring_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<shm_cpp::srv::SetSpring_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace shm_cpp
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetSpring_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetSpring_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetSpring_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace shm_cpp

namespace rosidl_generator_traits
{

[[deprecated("use shm_cpp::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const shm_cpp::srv::SetSpring_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  shm_cpp::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shm_cpp::srv::to_yaml() instead")]]
inline std::string to_yaml(const shm_cpp::srv::SetSpring_Response & msg)
{
  return shm_cpp::srv::to_yaml(msg);
}

template<>
inline const char * data_type<shm_cpp::srv::SetSpring_Response>()
{
  return "shm_cpp::srv::SetSpring_Response";
}

template<>
inline const char * name<shm_cpp::srv::SetSpring_Response>()
{
  return "shm_cpp/srv/SetSpring_Response";
}

template<>
struct has_fixed_size<shm_cpp::srv::SetSpring_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<shm_cpp::srv::SetSpring_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<shm_cpp::srv::SetSpring_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<shm_cpp::srv::SetSpring>()
{
  return "shm_cpp::srv::SetSpring";
}

template<>
inline const char * name<shm_cpp::srv::SetSpring>()
{
  return "shm_cpp/srv/SetSpring";
}

template<>
struct has_fixed_size<shm_cpp::srv::SetSpring>
  : std::integral_constant<
    bool,
    has_fixed_size<shm_cpp::srv::SetSpring_Request>::value &&
    has_fixed_size<shm_cpp::srv::SetSpring_Response>::value
  >
{
};

template<>
struct has_bounded_size<shm_cpp::srv::SetSpring>
  : std::integral_constant<
    bool,
    has_bounded_size<shm_cpp::srv::SetSpring_Request>::value &&
    has_bounded_size<shm_cpp::srv::SetSpring_Response>::value
  >
{
};

template<>
struct is_service<shm_cpp::srv::SetSpring>
  : std::true_type
{
};

template<>
struct is_service_request<shm_cpp::srv::SetSpring_Request>
  : std::true_type
{
};

template<>
struct is_service_response<shm_cpp::srv::SetSpring_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SHM_CPP__SRV__DETAIL__SET_SPRING__TRAITS_HPP_
