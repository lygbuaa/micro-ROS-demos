#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/pose_stamped.h>

#include <stdio.h>
#include <unistd.h>

// #define UROS_USE_FREERTOS_MEM_ALLOC

#ifdef UROS_USE_FREERTOS_MEM_ALLOC
#include "extra_source/extra_source.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#endif

#define ARRAY_LEN 128

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static const char* G_NODE_NAME_ = "pseudo_planning_node";

/* subscribe localization results */
static const char* G_LOCALIZATION_TOPIC_ = "/pseudo_localization_pose";
static const char* G_PLANNING_HEARTBEAT_ = "/planning/heartbeat";
static rcl_subscription_t g_subscriber_;
static geometry_msgs__msg__PoseStamped g_localization_pose_;
static std_msgs__msg__String g_hb_msg_;

/* pass destination pose to control node */
static const char* G_PLANNING_TOPIC_ = "/pseudo_planning_pose";
static rcl_publisher_t g_publisher_;
static rcl_publisher_t g_heartbeat_;
// geometry_msgs__msg__PoseStamped g_planning_pose_;

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	static int counter = 0;
	(void) last_call_time;
	if (timer != NULL) {
		sprintf(g_hb_msg_.data.data, "[%s] heartbeat: #%d", G_NODE_NAME_, counter++);
		g_hb_msg_.data.size = strlen(g_hb_msg_.data.data);
	    printf("%s\n", g_hb_msg_.data.data);
		RCSOFTCHECK(rcl_publish(&g_heartbeat_, &g_hb_msg_, NULL));
	}
}

static void localization_callback(const void * msg_ptr)
{
	static int counter = 0;
	const geometry_msgs__msg__PoseStamped * pose_ptr = (const geometry_msgs__msg__PoseStamped *)msg_ptr;
	if(counter % 100 == 0){
		printf("[%s] localization_callback count: #%d\n", G_NODE_NAME_, counter++);
	}
	RCSOFTCHECK(rcl_publish(&g_publisher_, pose_ptr, NULL));
}

static void rclc_support_init_lazy(rclc_support_t* ptr_support, int argc, const char * const * argv, rcl_allocator_t* ptr_allocator)
{
	while(rclc_support_init(ptr_support, argc, argv, ptr_allocator) != RCL_RET_OK){
		printf("rclc_support_init error: %s, sleep 1s and retry.\n", rcl_get_error_string().str);
		RCSOFTCHECK(rclc_support_fini(ptr_support));
#ifdef UROS_USE_FREERTOS_MEM_ALLOC
		vTaskDelay(pdMS_TO_TICKS(1000));
#else
		sleep(1);
#endif
	}
	printf("rclc_support_init success\n");
}

#ifdef UROS_USE_FREERTOS_MEM_ALLOC
int uros_pseudo_pnc_planning_demo(void)
#else
int main(int argc, const char * const * argv)
#endif
{
#ifdef UROS_USE_FREERTOS_MEM_ALLOC
	/* user freeRTOS memory alloc */
	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator))
		printf("error: set default allocator\n");
#endif

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
#ifdef UROS_USE_FREERTOS_MEM_ALLOC
	rclc_support_init_lazy(&support, 0, NULL, &allocator);
#else
	rclc_support_init_lazy(&support, argc, argv, &allocator);
#endif

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, G_NODE_NAME_, "", &support));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&g_subscriber_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
		G_LOCALIZATION_TOPIC_));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&g_publisher_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
		G_PLANNING_TOPIC_));

	// Fill the array with a known sequence
	g_hb_msg_.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	g_hb_msg_.data.size = 0;
	g_hb_msg_.data.capacity = ARRAY_LEN;

	// create heartbeat
	RCCHECK(rclc_publisher_init_default(
		&g_heartbeat_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
		G_PLANNING_HEARTBEAT_));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 5000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	printf("[%s] init topics success\n", G_NODE_NAME_);

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	/* number_of_handles = 2 */
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &g_subscriber_, &g_localization_pose_, &localization_callback, ON_NEW_DATA));
	printf("[%s] rclc_executor_spin...\n", G_NODE_NAME_);

	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&g_publisher_, &node))
	RCCHECK(rcl_publisher_fini(&g_heartbeat_, &node))
	RCCHECK(rcl_subscription_fini(&g_subscriber_, &node));
	RCCHECK(rcl_node_fini(&node))
	RCCHECK(rclc_support_fini(&support));

	return 0;
}
