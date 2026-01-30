// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from shm_cpp:srv/SetSpring.idl
// generated code does not contain a copyright notice

#ifndef SHM_CPP__SRV__DETAIL__SET_SPRING__STRUCT_H_
#define SHM_CPP__SRV__DETAIL__SET_SPRING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetSpring in the package shm_cpp.
typedef struct shm_cpp__srv__SetSpring_Request
{
  double k;
  double b;
} shm_cpp__srv__SetSpring_Request;

// Struct for a sequence of shm_cpp__srv__SetSpring_Request.
typedef struct shm_cpp__srv__SetSpring_Request__Sequence
{
  shm_cpp__srv__SetSpring_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} shm_cpp__srv__SetSpring_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetSpring in the package shm_cpp.
typedef struct shm_cpp__srv__SetSpring_Response
{
  bool success;
  rosidl_runtime_c__String message;
} shm_cpp__srv__SetSpring_Response;

// Struct for a sequence of shm_cpp__srv__SetSpring_Response.
typedef struct shm_cpp__srv__SetSpring_Response__Sequence
{
  shm_cpp__srv__SetSpring_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} shm_cpp__srv__SetSpring_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SHM_CPP__SRV__DETAIL__SET_SPRING__STRUCT_H_
