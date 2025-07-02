// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msgs:srv/AdultEvent.idl
// generated code does not contain a copyright notice

#ifndef MSGS__SRV__DETAIL__ADULT_EVENT__STRUCT_H_
#define MSGS__SRV__DETAIL__ADULT_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/AdultEvent in the package msgs.
typedef struct msgs__srv__AdultEvent_Request
{
  rosidl_runtime_c__String class_name;
} msgs__srv__AdultEvent_Request;

// Struct for a sequence of msgs__srv__AdultEvent_Request.
typedef struct msgs__srv__AdultEvent_Request__Sequence
{
  msgs__srv__AdultEvent_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msgs__srv__AdultEvent_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/AdultEvent in the package msgs.
typedef struct msgs__srv__AdultEvent_Response
{
  bool state_adult_event;
} msgs__srv__AdultEvent_Response;

// Struct for a sequence of msgs__srv__AdultEvent_Response.
typedef struct msgs__srv__AdultEvent_Response__Sequence
{
  msgs__srv__AdultEvent_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msgs__srv__AdultEvent_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSGS__SRV__DETAIL__ADULT_EVENT__STRUCT_H_
