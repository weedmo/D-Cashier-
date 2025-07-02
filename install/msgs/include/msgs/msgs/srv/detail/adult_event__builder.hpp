// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msgs:srv/AdultEvent.idl
// generated code does not contain a copyright notice

#ifndef MSGS__SRV__DETAIL__ADULT_EVENT__BUILDER_HPP_
#define MSGS__SRV__DETAIL__ADULT_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msgs/srv/detail/adult_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msgs
{

namespace srv
{

namespace builder
{

class Init_AdultEvent_Request_class_name
{
public:
  Init_AdultEvent_Request_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::msgs::srv::AdultEvent_Request class_name(::msgs::srv::AdultEvent_Request::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs::srv::AdultEvent_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs::srv::AdultEvent_Request>()
{
  return msgs::srv::builder::Init_AdultEvent_Request_class_name();
}

}  // namespace msgs


namespace msgs
{

namespace srv
{

namespace builder
{

class Init_AdultEvent_Response_state_adult_event
{
public:
  Init_AdultEvent_Response_state_adult_event()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::msgs::srv::AdultEvent_Response state_adult_event(::msgs::srv::AdultEvent_Response::_state_adult_event_type arg)
  {
    msg_.state_adult_event = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs::srv::AdultEvent_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs::srv::AdultEvent_Response>()
{
  return msgs::srv::builder::Init_AdultEvent_Response_state_adult_event();
}

}  // namespace msgs

#endif  // MSGS__SRV__DETAIL__ADULT_EVENT__BUILDER_HPP_
