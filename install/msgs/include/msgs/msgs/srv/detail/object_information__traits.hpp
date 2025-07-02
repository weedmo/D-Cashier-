// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msgs:srv/ObjectInformation.idl
// generated code does not contain a copyright notice

#ifndef MSGS__SRV__DETAIL__OBJECT_INFORMATION__TRAITS_HPP_
#define MSGS__SRV__DETAIL__OBJECT_INFORMATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msgs/srv/detail/object_information__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ObjectInformation_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: state_main
  {
    out << "state_main: ";
    rosidl_generator_traits::value_to_yaml(msg.state_main, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectInformation_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state_main
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_main: ";
    rosidl_generator_traits::value_to_yaml(msg.state_main, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectInformation_Request & msg, bool use_flow_style = false)
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
  const msgs::srv::ObjectInformation_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const msgs::srv::ObjectInformation_Request & msg)
{
  return msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<msgs::srv::ObjectInformation_Request>()
{
  return "msgs::srv::ObjectInformation_Request";
}

template<>
inline const char * name<msgs::srv::ObjectInformation_Request>()
{
  return "msgs/srv/ObjectInformation_Request";
}

template<>
struct has_fixed_size<msgs::srv::ObjectInformation_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msgs::srv::ObjectInformation_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msgs::srv::ObjectInformation_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ObjectInformation_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: nums
  {
    out << "nums: ";
    rosidl_generator_traits::value_to_yaml(msg.nums, out);
    out << ", ";
  }

  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << ", ";
  }

  // member: admin_event
  {
    out << "admin_event: ";
    rosidl_generator_traits::value_to_yaml(msg.admin_event, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectInformation_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: nums
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nums: ";
    rosidl_generator_traits::value_to_yaml(msg.nums, out);
    out << "\n";
  }

  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
  }

  // member: admin_event
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "admin_event: ";
    rosidl_generator_traits::value_to_yaml(msg.admin_event, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectInformation_Response & msg, bool use_flow_style = false)
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
  const msgs::srv::ObjectInformation_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const msgs::srv::ObjectInformation_Response & msg)
{
  return msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<msgs::srv::ObjectInformation_Response>()
{
  return "msgs::srv::ObjectInformation_Response";
}

template<>
inline const char * name<msgs::srv::ObjectInformation_Response>()
{
  return "msgs/srv/ObjectInformation_Response";
}

template<>
struct has_fixed_size<msgs::srv::ObjectInformation_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msgs::srv::ObjectInformation_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msgs::srv::ObjectInformation_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msgs::srv::ObjectInformation>()
{
  return "msgs::srv::ObjectInformation";
}

template<>
inline const char * name<msgs::srv::ObjectInformation>()
{
  return "msgs/srv/ObjectInformation";
}

template<>
struct has_fixed_size<msgs::srv::ObjectInformation>
  : std::integral_constant<
    bool,
    has_fixed_size<msgs::srv::ObjectInformation_Request>::value &&
    has_fixed_size<msgs::srv::ObjectInformation_Response>::value
  >
{
};

template<>
struct has_bounded_size<msgs::srv::ObjectInformation>
  : std::integral_constant<
    bool,
    has_bounded_size<msgs::srv::ObjectInformation_Request>::value &&
    has_bounded_size<msgs::srv::ObjectInformation_Response>::value
  >
{
};

template<>
struct is_service<msgs::srv::ObjectInformation>
  : std::true_type
{
};

template<>
struct is_service_request<msgs::srv::ObjectInformation_Request>
  : std::true_type
{
};

template<>
struct is_service_response<msgs::srv::ObjectInformation_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MSGS__SRV__DETAIL__OBJECT_INFORMATION__TRAITS_HPP_
