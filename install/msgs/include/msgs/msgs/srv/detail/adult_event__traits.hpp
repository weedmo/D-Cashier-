// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msgs:srv/AdultEvent.idl
// generated code does not contain a copyright notice

#ifndef MSGS__SRV__DETAIL__ADULT_EVENT__TRAITS_HPP_
#define MSGS__SRV__DETAIL__ADULT_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msgs/srv/detail/adult_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AdultEvent_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AdultEvent_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AdultEvent_Request & msg, bool use_flow_style = false)
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

}  // namespace msgs

namespace rosidl_generator_traits
{

[[deprecated("use msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msgs::srv::AdultEvent_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const msgs::srv::AdultEvent_Request & msg)
{
  return msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<msgs::srv::AdultEvent_Request>()
{
  return "msgs::srv::AdultEvent_Request";
}

template<>
inline const char * name<msgs::srv::AdultEvent_Request>()
{
  return "msgs/srv/AdultEvent_Request";
}

template<>
struct has_fixed_size<msgs::srv::AdultEvent_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msgs::srv::AdultEvent_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msgs::srv::AdultEvent_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AdultEvent_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: state_adult_event
  {
    out << "state_adult_event: ";
    rosidl_generator_traits::value_to_yaml(msg.state_adult_event, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AdultEvent_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state_adult_event
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_adult_event: ";
    rosidl_generator_traits::value_to_yaml(msg.state_adult_event, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AdultEvent_Response & msg, bool use_flow_style = false)
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

}  // namespace msgs

namespace rosidl_generator_traits
{

[[deprecated("use msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msgs::srv::AdultEvent_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const msgs::srv::AdultEvent_Response & msg)
{
  return msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<msgs::srv::AdultEvent_Response>()
{
  return "msgs::srv::AdultEvent_Response";
}

template<>
inline const char * name<msgs::srv::AdultEvent_Response>()
{
  return "msgs/srv/AdultEvent_Response";
}

template<>
struct has_fixed_size<msgs::srv::AdultEvent_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msgs::srv::AdultEvent_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msgs::srv::AdultEvent_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msgs::srv::AdultEvent>()
{
  return "msgs::srv::AdultEvent";
}

template<>
inline const char * name<msgs::srv::AdultEvent>()
{
  return "msgs/srv/AdultEvent";
}

template<>
struct has_fixed_size<msgs::srv::AdultEvent>
  : std::integral_constant<
    bool,
    has_fixed_size<msgs::srv::AdultEvent_Request>::value &&
    has_fixed_size<msgs::srv::AdultEvent_Response>::value
  >
{
};

template<>
struct has_bounded_size<msgs::srv::AdultEvent>
  : std::integral_constant<
    bool,
    has_bounded_size<msgs::srv::AdultEvent_Request>::value &&
    has_bounded_size<msgs::srv::AdultEvent_Response>::value
  >
{
};

template<>
struct is_service<msgs::srv::AdultEvent>
  : std::true_type
{
};

template<>
struct is_service_request<msgs::srv::AdultEvent_Request>
  : std::true_type
{
};

template<>
struct is_service_response<msgs::srv::AdultEvent_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MSGS__SRV__DETAIL__ADULT_EVENT__TRAITS_HPP_
