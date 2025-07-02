// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msgs:srv/ObjectInformation.idl
// generated code does not contain a copyright notice

#ifndef MSGS__SRV__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_
#define MSGS__SRV__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msgs/srv/detail/object_information__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msgs
{

namespace srv
{

namespace builder
{

class Init_ObjectInformation_Request_state_main
{
public:
  Init_ObjectInformation_Request_state_main()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::msgs::srv::ObjectInformation_Request state_main(::msgs::srv::ObjectInformation_Request::_state_main_type arg)
  {
    msg_.state_main = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs::srv::ObjectInformation_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs::srv::ObjectInformation_Request>()
{
  return msgs::srv::builder::Init_ObjectInformation_Request_state_main();
}

}  // namespace msgs


namespace msgs
{

namespace srv
{

namespace builder
{

class Init_ObjectInformation_Response_admin_event
{
public:
  explicit Init_ObjectInformation_Response_admin_event(::msgs::srv::ObjectInformation_Response & msg)
  : msg_(msg)
  {}
  ::msgs::srv::ObjectInformation_Response admin_event(::msgs::srv::ObjectInformation_Response::_admin_event_type arg)
  {
    msg_.admin_event = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs::srv::ObjectInformation_Response msg_;
};

class Init_ObjectInformation_Response_class_name
{
public:
  explicit Init_ObjectInformation_Response_class_name(::msgs::srv::ObjectInformation_Response & msg)
  : msg_(msg)
  {}
  Init_ObjectInformation_Response_admin_event class_name(::msgs::srv::ObjectInformation_Response::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_ObjectInformation_Response_admin_event(msg_);
  }

private:
  ::msgs::srv::ObjectInformation_Response msg_;
};

class Init_ObjectInformation_Response_nums
{
public:
  explicit Init_ObjectInformation_Response_nums(::msgs::srv::ObjectInformation_Response & msg)
  : msg_(msg)
  {}
  Init_ObjectInformation_Response_class_name nums(::msgs::srv::ObjectInformation_Response::_nums_type arg)
  {
    msg_.nums = std::move(arg);
    return Init_ObjectInformation_Response_class_name(msg_);
  }

private:
  ::msgs::srv::ObjectInformation_Response msg_;
};

class Init_ObjectInformation_Response_position
{
public:
  Init_ObjectInformation_Response_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectInformation_Response_nums position(::msgs::srv::ObjectInformation_Response::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_ObjectInformation_Response_nums(msg_);
  }

private:
  ::msgs::srv::ObjectInformation_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs::srv::ObjectInformation_Response>()
{
  return msgs::srv::builder::Init_ObjectInformation_Response_position();
}

}  // namespace msgs

#endif  // MSGS__SRV__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_
