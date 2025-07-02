// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msgs:srv/ObjectInformation.idl
// generated code does not contain a copyright notice

#ifndef MSGS__SRV__DETAIL__OBJECT_INFORMATION__STRUCT_HPP_
#define MSGS__SRV__DETAIL__OBJECT_INFORMATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msgs__srv__ObjectInformation_Request __attribute__((deprecated))
#else
# define DEPRECATED__msgs__srv__ObjectInformation_Request __declspec(deprecated)
#endif

namespace msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ObjectInformation_Request_
{
  using Type = ObjectInformation_Request_<ContainerAllocator>;

  explicit ObjectInformation_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state_main = false;
    }
  }

  explicit ObjectInformation_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state_main = false;
    }
  }

  // field types and members
  using _state_main_type =
    bool;
  _state_main_type state_main;

  // setters for named parameter idiom
  Type & set__state_main(
    const bool & _arg)
  {
    this->state_main = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs::srv::ObjectInformation_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs::srv::ObjectInformation_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs::srv::ObjectInformation_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs::srv::ObjectInformation_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs::srv::ObjectInformation_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs::srv::ObjectInformation_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs::srv::ObjectInformation_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs::srv::ObjectInformation_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs::srv::ObjectInformation_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs::srv::ObjectInformation_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs__srv__ObjectInformation_Request
    std::shared_ptr<msgs::srv::ObjectInformation_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs__srv__ObjectInformation_Request
    std::shared_ptr<msgs::srv::ObjectInformation_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectInformation_Request_ & other) const
  {
    if (this->state_main != other.state_main) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectInformation_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectInformation_Request_

// alias to use template instance with default allocator
using ObjectInformation_Request =
  msgs::srv::ObjectInformation_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace msgs


#ifndef _WIN32
# define DEPRECATED__msgs__srv__ObjectInformation_Response __attribute__((deprecated))
#else
# define DEPRECATED__msgs__srv__ObjectInformation_Response __declspec(deprecated)
#endif

namespace msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ObjectInformation_Response_
{
  using Type = ObjectInformation_Response_<ContainerAllocator>;

  explicit ObjectInformation_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nums = 0ll;
      this->class_name = "";
      this->admin_event = 0ll;
    }
  }

  explicit ObjectInformation_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nums = 0ll;
      this->class_name = "";
      this->admin_event = 0ll;
    }
  }

  // field types and members
  using _position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _position_type position;
  using _nums_type =
    int64_t;
  _nums_type nums;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _admin_event_type =
    int64_t;
  _admin_event_type admin_event;

  // setters for named parameter idiom
  Type & set__position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__nums(
    const int64_t & _arg)
  {
    this->nums = _arg;
    return *this;
  }
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__admin_event(
    const int64_t & _arg)
  {
    this->admin_event = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs::srv::ObjectInformation_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs::srv::ObjectInformation_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs::srv::ObjectInformation_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs::srv::ObjectInformation_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs::srv::ObjectInformation_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs::srv::ObjectInformation_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs::srv::ObjectInformation_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs::srv::ObjectInformation_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs::srv::ObjectInformation_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs::srv::ObjectInformation_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs__srv__ObjectInformation_Response
    std::shared_ptr<msgs::srv::ObjectInformation_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs__srv__ObjectInformation_Response
    std::shared_ptr<msgs::srv::ObjectInformation_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectInformation_Response_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->nums != other.nums) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->admin_event != other.admin_event) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectInformation_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectInformation_Response_

// alias to use template instance with default allocator
using ObjectInformation_Response =
  msgs::srv::ObjectInformation_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace msgs

namespace msgs
{

namespace srv
{

struct ObjectInformation
{
  using Request = msgs::srv::ObjectInformation_Request;
  using Response = msgs::srv::ObjectInformation_Response;
};

}  // namespace srv

}  // namespace msgs

#endif  // MSGS__SRV__DETAIL__OBJECT_INFORMATION__STRUCT_HPP_
