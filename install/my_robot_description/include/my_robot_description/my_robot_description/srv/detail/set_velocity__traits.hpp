// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_description:srv/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_DESCRIPTION__SRV__DETAIL__SET_VELOCITY__TRAITS_HPP_
#define MY_ROBOT_DESCRIPTION__SRV__DETAIL__SET_VELOCITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_description/srv/detail/set_velocity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_description
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetVelocity_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: linear_x
  {
    out << "linear_x: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_x, out);
    out << ", ";
  }

  // member: angular_z
  {
    out << "angular_z: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetVelocity_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: linear_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_x: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_x, out);
    out << "\n";
  }

  // member: angular_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_z: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetVelocity_Request & msg, bool use_flow_style = false)
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

}  // namespace my_robot_description

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_description::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_description::srv::SetVelocity_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_description::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_description::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_description::srv::SetVelocity_Request & msg)
{
  return my_robot_description::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_description::srv::SetVelocity_Request>()
{
  return "my_robot_description::srv::SetVelocity_Request";
}

template<>
inline const char * name<my_robot_description::srv::SetVelocity_Request>()
{
  return "my_robot_description/srv/SetVelocity_Request";
}

template<>
struct has_fixed_size<my_robot_description::srv::SetVelocity_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_robot_description::srv::SetVelocity_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_robot_description::srv::SetVelocity_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace my_robot_description
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetVelocity_Response & msg,
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
  const SetVelocity_Response & msg,
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

inline std::string to_yaml(const SetVelocity_Response & msg, bool use_flow_style = false)
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

}  // namespace my_robot_description

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_description::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_description::srv::SetVelocity_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_description::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_description::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_description::srv::SetVelocity_Response & msg)
{
  return my_robot_description::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_description::srv::SetVelocity_Response>()
{
  return "my_robot_description::srv::SetVelocity_Response";
}

template<>
inline const char * name<my_robot_description::srv::SetVelocity_Response>()
{
  return "my_robot_description/srv/SetVelocity_Response";
}

template<>
struct has_fixed_size<my_robot_description::srv::SetVelocity_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_robot_description::srv::SetVelocity_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_robot_description::srv::SetVelocity_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<my_robot_description::srv::SetVelocity>()
{
  return "my_robot_description::srv::SetVelocity";
}

template<>
inline const char * name<my_robot_description::srv::SetVelocity>()
{
  return "my_robot_description/srv/SetVelocity";
}

template<>
struct has_fixed_size<my_robot_description::srv::SetVelocity>
  : std::integral_constant<
    bool,
    has_fixed_size<my_robot_description::srv::SetVelocity_Request>::value &&
    has_fixed_size<my_robot_description::srv::SetVelocity_Response>::value
  >
{
};

template<>
struct has_bounded_size<my_robot_description::srv::SetVelocity>
  : std::integral_constant<
    bool,
    has_bounded_size<my_robot_description::srv::SetVelocity_Request>::value &&
    has_bounded_size<my_robot_description::srv::SetVelocity_Response>::value
  >
{
};

template<>
struct is_service<my_robot_description::srv::SetVelocity>
  : std::true_type
{
};

template<>
struct is_service_request<my_robot_description::srv::SetVelocity_Request>
  : std::true_type
{
};

template<>
struct is_service_response<my_robot_description::srv::SetVelocity_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_DESCRIPTION__SRV__DETAIL__SET_VELOCITY__TRAITS_HPP_
