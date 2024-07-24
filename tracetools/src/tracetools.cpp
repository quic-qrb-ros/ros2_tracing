// Copyright 2019 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tracetools/tracetools.h"

#ifndef TRACETOOLS_DISABLED

#ifdef TRACETOOLS_PERFETTO_ENABLED
# include "tracetools/perfetto_trace.h"
#endif

bool ros_trace_compile_status()
{
#if defined(TRACETOOLS_PERFETTO_ENABLED)
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4100)
#endif

void TRACEPOINT(
  rcl_init,
  const void * context_handle)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rcl_init",
    "context_handle", context_handle);
#endif
}

void TRACEPOINT(
  rcl_node_init,
  const void * node_handle,
  const void * rmw_handle,
  const char * node_name,
  const char * node_namespace)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rcl_node_init",
    "node_handle", node_handle,
    "rmw_handle", rmw_handle,
    "node_name", node_name,
    "node_namespace", node_namespace);
#endif
}

void TRACEPOINT(
  rcl_publisher_init,
  const void * publisher_handle,
  const void * node_handle,
  const void * rmw_publisher_handle,
  const char * topic_name,
  const size_t queue_depth)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rcl_publisher_init",
    "publisher_handle", publisher_handle,
    "node_handle", node_handle,
    "rmw_publisher_handle", rmw_publisher_handle,
    "topic_name", topic_name,
    "queue_depth", queue_depth);
#endif
}

void TRACEPOINT(
  rcl_subscription_init,
  const void * subscription_handle,
  const void * node_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t queue_depth)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rcl_subscription_init",
    "subscription_handle", subscription_handle,
    "node_handle", node_handle,
    "rmw_subscription_handle", rmw_subscription_handle,
    "topic_name", topic_name,
    "queue_depth", queue_depth);
#endif
}

void TRACEPOINT(
  rclcpp_subscription_init,
  const void * subscription_handle,
  const void * subscription)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rclcpp_subscription_init",
    "subscription_handle", subscription_handle,
    "subscription", subscription);
#endif
}

void TRACEPOINT(
  rclcpp_subscription_callback_added,
  const void * subscription,
  const void * callback)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rclcpp_subscription_callback_added",
    "subscription", subscription,
    "callback", callback);
#endif
}

void TRACEPOINT(
  rcl_service_init,
  const void * service_handle,
  const void * node_handle,
  const void * rmw_service_handle,
  const char * service_name)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rcl_service_init",
    "service_handle", service_handle,
    "node_handle", node_handle,
    "rmw_service_handle", rmw_service_handle,
    "service_name", service_name);
#endif
}

void TRACEPOINT(
  rclcpp_service_callback_added,
  const void * service_handle,
  const void * callback)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rclcpp_service_callback_added",
    "service_handle", service_handle,
    "callback", callback);
#endif
}

void TRACEPOINT(
  rcl_client_init,
  const void * client_handle,
  const void * node_handle,
  const void * rmw_client_handle,
  const char * service_name)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rcl_client_init",
    "client_handle", client_handle,
    "node_handle", node_handle,
    "rmw_client_handle", rmw_client_handle,
    "service_name", service_name);
#endif
}

void TRACEPOINT(
  rcl_timer_init,
  const void * timer_handle,
  int64_t period)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rcl_timer_init",
    "timer_handle", timer_handle,
    "period", period);
#endif
}

void TRACEPOINT(
  rclcpp_timer_callback_added,
  const void * timer_handle,
  const void * callback)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rclcpp_timer_callback_added",
    "timer_handle", timer_handle,
    "callback", callback);
#endif
}

void TRACEPOINT(
  rclcpp_callback_register,
  const void * callback,
  const char * function_symbol)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "rclcpp_callback_register",
    "callback", callback,
    "function_symbol", function_symbol);
#endif
}

void TRACEPOINT(
  callback_start,
  const void * callback,
  const bool is_intra_process)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "callback_start",
    "callback", callback,
    "is_intra_process", is_intra_process);
#endif
}

void TRACEPOINT(
  callback_end,
  const void * callback)
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
  TRACE_EVENT(
    "ros2",
    "callback_end",
    "callback", callback);
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#endif  // TRACETOOLS_DISABLED
