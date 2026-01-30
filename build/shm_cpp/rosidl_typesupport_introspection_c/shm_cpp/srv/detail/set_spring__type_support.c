// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from shm_cpp:srv/SetSpring.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "shm_cpp/srv/detail/set_spring__rosidl_typesupport_introspection_c.h"
#include "shm_cpp/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "shm_cpp/srv/detail/set_spring__functions.h"
#include "shm_cpp/srv/detail/set_spring__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  shm_cpp__srv__SetSpring_Request__init(message_memory);
}

void shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_fini_function(void * message_memory)
{
  shm_cpp__srv__SetSpring_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_message_member_array[2] = {
  {
    "k",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shm_cpp__srv__SetSpring_Request, k),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "b",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shm_cpp__srv__SetSpring_Request, b),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_message_members = {
  "shm_cpp__srv",  // message namespace
  "SetSpring_Request",  // message name
  2,  // number of fields
  sizeof(shm_cpp__srv__SetSpring_Request),
  shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_message_member_array,  // message members
  shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_message_type_support_handle = {
  0,
  &shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_shm_cpp
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shm_cpp, srv, SetSpring_Request)() {
  if (!shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_message_type_support_handle.typesupport_identifier) {
    shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &shm_cpp__srv__SetSpring_Request__rosidl_typesupport_introspection_c__SetSpring_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "shm_cpp/srv/detail/set_spring__rosidl_typesupport_introspection_c.h"
// already included above
// #include "shm_cpp/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "shm_cpp/srv/detail/set_spring__functions.h"
// already included above
// #include "shm_cpp/srv/detail/set_spring__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  shm_cpp__srv__SetSpring_Response__init(message_memory);
}

void shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_fini_function(void * message_memory)
{
  shm_cpp__srv__SetSpring_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shm_cpp__srv__SetSpring_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shm_cpp__srv__SetSpring_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_message_members = {
  "shm_cpp__srv",  // message namespace
  "SetSpring_Response",  // message name
  2,  // number of fields
  sizeof(shm_cpp__srv__SetSpring_Response),
  shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_message_member_array,  // message members
  shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_message_type_support_handle = {
  0,
  &shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_shm_cpp
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shm_cpp, srv, SetSpring_Response)() {
  if (!shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_message_type_support_handle.typesupport_identifier) {
    shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &shm_cpp__srv__SetSpring_Response__rosidl_typesupport_introspection_c__SetSpring_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "shm_cpp/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "shm_cpp/srv/detail/set_spring__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_service_members = {
  "shm_cpp__srv",  // service namespace
  "SetSpring",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_Request_message_type_support_handle,
  NULL  // response message
  // shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_Response_message_type_support_handle
};

static rosidl_service_type_support_t shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_service_type_support_handle = {
  0,
  &shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shm_cpp, srv, SetSpring_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shm_cpp, srv, SetSpring_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_shm_cpp
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shm_cpp, srv, SetSpring)() {
  if (!shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_service_type_support_handle.typesupport_identifier) {
    shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shm_cpp, srv, SetSpring_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shm_cpp, srv, SetSpring_Response)()->data;
  }

  return &shm_cpp__srv__detail__set_spring__rosidl_typesupport_introspection_c__SetSpring_service_type_support_handle;
}
