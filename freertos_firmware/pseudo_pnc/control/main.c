#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <carla_msgs/msg/carla_ego_vehicle_control.h>

#include <stdio.h>
#include <unistd.h>

#define ARRAY_LEN 200

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){fprintf(stderr, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){fprintf(stderr, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

const char* G_NODE_NAME_ = "pseudo_control_node";

/* subscribe planning results */
const char* G_PLANNING_TOPIC_ = "/pseudo_planning_pose";
rcl_subscription_t g_subscriber_;
geometry_msgs__msg__PoseStamped g_planning_pose_;

/* publish control command */
const char* G_CONTROL_TOPIC_ = "/carla/ego_vehicle/vehicle_control_cmd";
rcl_publisher_t g_publisher_;
carla_msgs__msg__CarlaEgoVehicleControl g_control_cmd_;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	static int counter = 0;
	(void) last_call_time;
	if (timer != NULL) {
	    fprintf(stderr, "[%s] heartbeat: #%d\n", G_NODE_NAME_, counter++);
	}
}

void planning_callback(const void * msg_ptr)
{
	static int counter = 0;
	const geometry_msgs__msg__PoseStamped * pose_ptr = (const geometry_msgs__msg__PoseStamped *)msg_ptr;
	if(counter % 100 == 0){
		fprintf(stderr, "[%s] planning_callback count: #%d\n", G_NODE_NAME_, counter++);
	}
	g_control_cmd_.header = pose_ptr->header;
	g_control_cmd_.throttle = pose_ptr->pose.orientation.x;
	g_control_cmd_.steer = pose_ptr->pose.orientation.y;
	g_control_cmd_.brake = pose_ptr->pose.orientation.z;
	g_control_cmd_.gear = pose_ptr->pose.orientation.w;
	g_control_cmd_.reverse = (g_control_cmd_.gear < 0);
	RCSOFTCHECK(rcl_publish(&g_publisher_, &g_control_cmd_, NULL));
}

int main(int argc, const char * const * argv)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, argc, argv, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, G_NODE_NAME_, "", &support));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&g_subscriber_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
		G_PLANNING_TOPIC_));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&g_publisher_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(carla_msgs, msg, CarlaEgoVehicleControl),
		G_CONTROL_TOPIC_));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 5000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	/* number_of_handles = 2 */
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &g_subscriber_, &g_planning_pose_, &planning_callback, ON_NEW_DATA));

	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&g_publisher_, &node))
	RCCHECK(rcl_subscription_fini(&g_subscriber_, &node));
	RCCHECK(rcl_node_fini(&node))
}
