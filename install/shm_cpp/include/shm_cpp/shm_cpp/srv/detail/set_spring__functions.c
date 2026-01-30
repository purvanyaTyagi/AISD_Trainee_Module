// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from shm_cpp:srv/SetSpring.idl
// generated code does not contain a copyright notice
#include "shm_cpp/srv/detail/set_spring__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
shm_cpp__srv__SetSpring_Request__init(shm_cpp__srv__SetSpring_Request * msg)
{
  if (!msg) {
    return false;
  }
  // k
  // b
  return true;
}

void
shm_cpp__srv__SetSpring_Request__fini(shm_cpp__srv__SetSpring_Request * msg)
{
  if (!msg) {
    return;
  }
  // k
  // b
}

bool
shm_cpp__srv__SetSpring_Request__are_equal(const shm_cpp__srv__SetSpring_Request * lhs, const shm_cpp__srv__SetSpring_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // k
  if (lhs->k != rhs->k) {
    return false;
  }
  // b
  if (lhs->b != rhs->b) {
    return false;
  }
  return true;
}

bool
shm_cpp__srv__SetSpring_Request__copy(
  const shm_cpp__srv__SetSpring_Request * input,
  shm_cpp__srv__SetSpring_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // k
  output->k = input->k;
  // b
  output->b = input->b;
  return true;
}

shm_cpp__srv__SetSpring_Request *
shm_cpp__srv__SetSpring_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shm_cpp__srv__SetSpring_Request * msg = (shm_cpp__srv__SetSpring_Request *)allocator.allocate(sizeof(shm_cpp__srv__SetSpring_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(shm_cpp__srv__SetSpring_Request));
  bool success = shm_cpp__srv__SetSpring_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
shm_cpp__srv__SetSpring_Request__destroy(shm_cpp__srv__SetSpring_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    shm_cpp__srv__SetSpring_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
shm_cpp__srv__SetSpring_Request__Sequence__init(shm_cpp__srv__SetSpring_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shm_cpp__srv__SetSpring_Request * data = NULL;

  if (size) {
    data = (shm_cpp__srv__SetSpring_Request *)allocator.zero_allocate(size, sizeof(shm_cpp__srv__SetSpring_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = shm_cpp__srv__SetSpring_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        shm_cpp__srv__SetSpring_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
shm_cpp__srv__SetSpring_Request__Sequence__fini(shm_cpp__srv__SetSpring_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      shm_cpp__srv__SetSpring_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

shm_cpp__srv__SetSpring_Request__Sequence *
shm_cpp__srv__SetSpring_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shm_cpp__srv__SetSpring_Request__Sequence * array = (shm_cpp__srv__SetSpring_Request__Sequence *)allocator.allocate(sizeof(shm_cpp__srv__SetSpring_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = shm_cpp__srv__SetSpring_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
shm_cpp__srv__SetSpring_Request__Sequence__destroy(shm_cpp__srv__SetSpring_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    shm_cpp__srv__SetSpring_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
shm_cpp__srv__SetSpring_Request__Sequence__are_equal(const shm_cpp__srv__SetSpring_Request__Sequence * lhs, const shm_cpp__srv__SetSpring_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!shm_cpp__srv__SetSpring_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
shm_cpp__srv__SetSpring_Request__Sequence__copy(
  const shm_cpp__srv__SetSpring_Request__Sequence * input,
  shm_cpp__srv__SetSpring_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(shm_cpp__srv__SetSpring_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    shm_cpp__srv__SetSpring_Request * data =
      (shm_cpp__srv__SetSpring_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!shm_cpp__srv__SetSpring_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          shm_cpp__srv__SetSpring_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!shm_cpp__srv__SetSpring_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
shm_cpp__srv__SetSpring_Response__init(shm_cpp__srv__SetSpring_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    shm_cpp__srv__SetSpring_Response__fini(msg);
    return false;
  }
  return true;
}

void
shm_cpp__srv__SetSpring_Response__fini(shm_cpp__srv__SetSpring_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
shm_cpp__srv__SetSpring_Response__are_equal(const shm_cpp__srv__SetSpring_Response * lhs, const shm_cpp__srv__SetSpring_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
shm_cpp__srv__SetSpring_Response__copy(
  const shm_cpp__srv__SetSpring_Response * input,
  shm_cpp__srv__SetSpring_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

shm_cpp__srv__SetSpring_Response *
shm_cpp__srv__SetSpring_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shm_cpp__srv__SetSpring_Response * msg = (shm_cpp__srv__SetSpring_Response *)allocator.allocate(sizeof(shm_cpp__srv__SetSpring_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(shm_cpp__srv__SetSpring_Response));
  bool success = shm_cpp__srv__SetSpring_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
shm_cpp__srv__SetSpring_Response__destroy(shm_cpp__srv__SetSpring_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    shm_cpp__srv__SetSpring_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
shm_cpp__srv__SetSpring_Response__Sequence__init(shm_cpp__srv__SetSpring_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shm_cpp__srv__SetSpring_Response * data = NULL;

  if (size) {
    data = (shm_cpp__srv__SetSpring_Response *)allocator.zero_allocate(size, sizeof(shm_cpp__srv__SetSpring_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = shm_cpp__srv__SetSpring_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        shm_cpp__srv__SetSpring_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
shm_cpp__srv__SetSpring_Response__Sequence__fini(shm_cpp__srv__SetSpring_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      shm_cpp__srv__SetSpring_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

shm_cpp__srv__SetSpring_Response__Sequence *
shm_cpp__srv__SetSpring_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shm_cpp__srv__SetSpring_Response__Sequence * array = (shm_cpp__srv__SetSpring_Response__Sequence *)allocator.allocate(sizeof(shm_cpp__srv__SetSpring_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = shm_cpp__srv__SetSpring_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
shm_cpp__srv__SetSpring_Response__Sequence__destroy(shm_cpp__srv__SetSpring_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    shm_cpp__srv__SetSpring_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
shm_cpp__srv__SetSpring_Response__Sequence__are_equal(const shm_cpp__srv__SetSpring_Response__Sequence * lhs, const shm_cpp__srv__SetSpring_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!shm_cpp__srv__SetSpring_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
shm_cpp__srv__SetSpring_Response__Sequence__copy(
  const shm_cpp__srv__SetSpring_Response__Sequence * input,
  shm_cpp__srv__SetSpring_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(shm_cpp__srv__SetSpring_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    shm_cpp__srv__SetSpring_Response * data =
      (shm_cpp__srv__SetSpring_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!shm_cpp__srv__SetSpring_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          shm_cpp__srv__SetSpring_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!shm_cpp__srv__SetSpring_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
