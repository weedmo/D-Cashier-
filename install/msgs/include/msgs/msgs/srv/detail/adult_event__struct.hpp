// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msgs:srv/AdultEvent.idl
// generated code does not contain a copyright notice

#ifndef MSGS__SRV__DETAIL__ADULT_EVENT__STRUCT_HPP_
#define MSGS__SRV__DETAIL__ADULT_EVENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msgs__srv__AdultEvent_Request __attribute__((deprecated))
#else
# define DEPRECATED__msgs__srv__AdultEvent_Request __declspec(deprecated)
#endif

namespace msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AdultEvent_Request_
{
  using Type = AdultEvent_Request_<ContainerAllocator>;

  explicit AdultEvent_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
    }
  }

  explicit AdultEvent_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
    }
  }

  // field types and members
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;

  // setters for named parameter idiom
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs::srv::AdultEvent_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs::srv::AdultEvent_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs::srv::AdultEvent_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs::srv::AdultEvent_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs::srv::AdultEvent_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs::srv::AdultEvent_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs::srv::AdultEvent_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs::srv::AdultEvent_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs::srv::AdultEvent_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs::srv::AdultEvent_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs__srv__AdultEvent_Request
    std::shared_ptr<msgs::srv::AdultEvent_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs__srv__AdultEvent_Request
    std::shared_ptr<msgs::srv::AdultEvent_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AdultEvent_Request_ & other) const
  {
    if (this->class_name != other.class_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const AdultEvent_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AdultEvent_Request_

// alias to use template instance with default allocator
using AdultEvent_Request =
  msgs::srv::AdultEvent_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace msgs


#ifndef _WIN32
# define DEPRECATED__msgs__srv__AdultEvent_Response __attribute__((deprecated))
#else
# define DEPRECATED__msgs__srv__AdultEvent_Response __declspec(deprecated)
#endif

namespace msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AdultEvent_Response_
{
  using Type = AdultEvent_Response_<ContainerAllocator>;

  explicit AdultEvent_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state_adult_event = false;
    }
  }

  explicit AdultEvent_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state_adult_event = false;
    }
  }

  // field types and members
  using _state_adult_event_type =
    bool;
  _state_adult_event_type state_adult_event;

  // setters for named parameter idiom
  Type & set__state_adult_event(
    const bool & _arg)
  {
    this->state_adult_event = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs::srv::AdultEvent_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs::srv::AdultEvent_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs::srv::AdultEvent_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs::srv::AdultEvent_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs::srv::AdultEvent_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs::srv::AdultEvent_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs::srv::AdultEvent_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs::srv::AdultEvent_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs::srv::AdultEvent_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs::srv::AdultEvent_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs__srv__AdultEvent_Response
    std::shared_ptr<msgs::srv::AdultEvent_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs__srv__AdultEvent_Response
    std::shared_ptr<msgs::srv::AdultEvent_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AdultEvent_Response_ & other) const
  {
    if (this->state_adult_event != other.state_adult_event) {
      return false;
    }
    return true;
  }
  bool operator!=(const AdultEvent_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AdultEvent_Response_

// alias to use template instance with default allocator
using AdultEvent_Response =
  msgs::srv::AdultEvent_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace msgs

namespace msgs
{

namespace srv
{

struct AdultEvent
{
  using Request = msgs::srv::AdultEvent_Request;
  using Response = msgs::srv::AdultEvent_Response;
};

}  // namespace srv

}  // namespace msgs

#endif  // MSGS__SRV__DETAIL__ADULT_EVENT__STRUCT_HPP_
