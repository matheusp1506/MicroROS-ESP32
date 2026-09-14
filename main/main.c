#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

// micro-ROS headers
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// ROS 2 Messages
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

static const char *TAG = "JUCA_MICROROS";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGW(TAG, "Soft check failed on line %d: %d.", __LINE__, (int)temp_rc); }}

static void init_covariance_matrices(nav_msgs__msg__Odometry *odom, sensor_msgs__msg__Imu *imu) {
    static char odom_frame_id_storage[16] = "odom";
    static char odom_child_frame_id_storage[16] = "base_link";
    static char imu_frame_id_storage[16] = "base_link";

    // Assign static buffers to Odometry strings
    odom->header.frame_id.data = odom_frame_id_storage;
    odom->header.frame_id.size = strlen(odom_frame_id_storage);
    odom->header.frame_id.capacity = sizeof(odom_frame_id_storage);

    odom->child_frame_id.data = odom_child_frame_id_storage;
    odom->child_frame_id.size = strlen(odom_child_frame_id_storage);
    odom->child_frame_id.capacity = sizeof(odom_child_frame_id_storage);

    // Assign static buffers to IMU string
    imu->header.frame_id.data = imu_frame_id_storage;
    imu->header.frame_id.size = strlen(imu_frame_id_storage);
    imu->header.frame_id.capacity = sizeof(imu_frame_id_storage);

    // Set covariances (6x6 = 36 elements)
    memset(odom->pose.covariance, 0, sizeof(odom->pose.covariance));
    memset(odom->twist.covariance, 0, sizeof(odom->twist.covariance));
    odom->pose.covariance[0] = 0.01;
    odom->pose.covariance[7] = 0.01;
    odom->pose.covariance[35] = 0.02418;
    odom->twist.covariance[0] = 0.1;
    odom->twist.covariance[35] = 0.05;

    memset(imu->orientation_covariance, 0, sizeof(imu->orientation_covariance));
    memset(imu->angular_velocity_covariance, 0, sizeof(imu->angular_velocity_covariance));
    memset(imu->linear_acceleration_covariance, 0, sizeof(imu->linear_acceleration_covariance));
    imu->orientation_covariance[0] = 99999.0;
    imu->orientation_covariance[4] = 99999.0;
    imu->orientation_covariance[8] = 0.0001998;
    imu->angular_velocity_covariance[0] = 99999.0;
    imu->angular_velocity_covariance[4] = 99999.0;
    imu->angular_velocity_covariance[8] = 0.0001998;
}

static void microros_task(void *pvParameters) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // 1. Initialize options and inject the Kconfig UDP address
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    // 2. Initialize support with the configured RMW options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    const int timeout_ms = 1000;
    while (rmw_uros_ping_agent(timeout_ms, 1) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Waiting for micro-ROS agent handshake...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Agent handshake acknowledged! Initializing node...");

    // 3. Create node
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "juca_esp32_node", "", &support));

    // 4. Initialize Publishers
    rcl_publisher_t odom_pub;
    rcl_publisher_t imu_pub;

    RCCHECK(rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"
    ));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));

    nav_msgs__msg__Odometry odom_msg;
    sensor_msgs__msg__Imu imu_msg;
    memset(&odom_msg, 0, sizeof(nav_msgs__msg__Odometry));
    memset(&imu_msg, 0, sizeof(sensor_msgs__msg__Imu));
    init_covariance_matrices(&odom_msg, &imu_msg);

    RCSOFTCHECK(rmw_uros_sync_session(1000));

    ESP_LOGI(TAG, "micro-ROS initialized with Agent at %s:%s. Streaming simulated spin...", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 Hz
    const float dt = 0.05f;                          // 50 ms loop interval
    const float spin_speed = 1.0f;                   // Angular velocity: 1.0 rad/s (~57 deg/s)

    float yaw = 0.0f;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Advance simulated yaw angle
        yaw += spin_speed * dt;
        if (yaw > 2.0f * (float)M_PI) {
            yaw -= 2.0f * (float)M_PI;
        }

        // Convert yaw to quaternion (rotation around Z axis)
        float qz = sinf(yaw * 0.5f);
        float qw = cosf(yaw * 0.5f);

        int64_t time_ns = rmw_uros_epoch_nanos();
        int32_t sec = time_ns / 1000000000;
        uint32_t nanosec = time_ns % 1000000000;

        // Populate Odometry
        odom_msg.header.stamp.sec = sec;
        odom_msg.header.stamp.nanosec = nanosec;
        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 0.15;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = qz;
        odom_msg.pose.pose.orientation.w = qw;
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.angular.z = spin_speed;

        rcl_ret_t odom_ret = rcl_publish(&odom_pub, &odom_msg, NULL);
        if (odom_ret != RCL_RET_OK) {
            ESP_LOGE(TAG, "rcl_publish odom failed! rc = %d", (int)odom_ret);
        }

        // Populate IMU
        imu_msg.header.stamp.sec = sec;
        imu_msg.header.stamp.nanosec = nanosec;
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = qz;
        imu_msg.orientation.w = qw;
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = spin_speed;
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 9.81;

        rcl_ret_t imu_ret = rcl_publish(&imu_pub, &imu_msg, NULL);
        if (imu_ret != RCL_RET_OK) {
            ESP_LOGE(TAG, "rcl_publish IMU failed! rc = %d", (int)imu_ret);
        }
    }

    vTaskDelete(NULL);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreatePinnedToCore(
        microros_task,
        "microros_task",
        16384,
        NULL,
        5,
        NULL,
        1
    );
}