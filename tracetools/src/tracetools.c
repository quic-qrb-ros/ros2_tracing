#include "tracetools/tracetools.h"

#ifdef WITH_LTTNG
#include "tp_call.h"
#endif


bool ros_trace_compile_status()
{
#ifdef WITH_LTTNG
  return true;
#else
  return false;
#endif
}

void ros_trace_rcl_init()
{
#ifdef WITH_LTTNG
  tracepoint(ros2, rcl_init);
#endif
}

void ros_trace_rcl_node_init(const char * node_name, const char * node_namespace, const void * rmw_handle)
{
#ifdef WITH_LTTNG
  tracepoint(ros2, rcl_node_init, node_name, node_namespace, rmw_handle);
#endif
}

void ros_trace_rcl_publisher_init(const char * node_name, const char * node_namespace)
{
#ifdef WITH_LTTNG
  tracepoint(ros2, rcl_publisher_init, node_name, node_namespace);
#endif
}

void ros_trace_rcl_subscription_init(const char * node_name, const char * topic_name)
{
#ifdef WITH_LTTNG
  tracepoint(ros2, rcl_subscription_init, node_name, topic_name);
#endif
}

void ros_trace_rclcpp_subscription_callback_start(const void * callback, const bool is_intra_process)
{
#ifdef WITH_LTTNG
  tracepoint(ros2, rclcpp_subscription_callback_start, callback, (is_intra_process ? 1 : 0));
#endif
}

void ros_trace_rclcpp_subscription_callback_end(const void * callback)
{
#ifdef WITH_LTTNG
  tracepoint(ros2, rclcpp_subscription_callback_end, callback);
#endif
}

void ros_trace_rclcpp_service_callback_start(const void * callback)
{
#ifdef WITH_LTTNG
  tracepoint(ros2, rclcpp_service_callback_start, callback);
#endif
}

void ros_trace_rclcpp_service_callback_end(const void * callback)
{
#ifdef WITH_LTTNG
  tracepoint(ros2, rclcpp_service_callback_end, callback);
#endif
}
