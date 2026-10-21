#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

// micro-ROS headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

// ROS 2 Messages
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

static const char *TAG = "JUCA_MICROROS";

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t wifi_event_group;

// Wi-Fi and micro-ROS agent configuration is managed via Kconfig (idf.py menuconfig).
// Values will be provided by sdkconfig (CONFIG_*).
#ifndef CONFIG_ESP_WIFI_SSID
#define CONFIG_ESP_WIFI_SSID "YOUR_WIFI_SSID"
#endif
#ifndef CONFIG_ESP_WIFI_PASSWORD
#define CONFIG_ESP_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif
#ifndef CONFIG_MICRO_ROS_AGENT_IP
#define CONFIG_MICRO_ROS_AGENT_IP "192.168.1.100"
#endif
#ifndef CONFIG_MICRO_ROS_AGENT_PORT
#define CONFIG_MICRO_ROS_AGENT_PORT "8888"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGW(TAG, "Soft check failed on line %d: %d.", __LINE__, (int)temp_rc); }}

#include "lwip/netdb.h"

// Satisfy libmicroros.a symbols by routing directly to lwIP's POSIX functions
int lwip_getaddrinfo(const char *nodename, const char *servname,
                     const struct addrinfo *hints, struct addrinfo **res) {
    return getaddrinfo(nodename, servname, hints, res);
}

void lwip_freeaddrinfo(struct addrinfo *ai) {
    freeaddrinfo(ai);
}

/* =========================================================================
 * Custom POSIX UDP Transport for micro-ROS
 * ========================================================================= */
typedef struct {
    struct sockaddr_in agent_addr;
    int sock;
} custom_transport_args_t;

static bool custom_transport_open(struct uxrCustomTransport *transport) {
    custom_transport_args_t *args = (custom_transport_args_t *)transport->args;
    args->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    return args->sock >= 0;
}

static bool custom_transport_close(struct uxrCustomTransport *transport) {
    custom_transport_args_t *args = (custom_transport_args_t *)transport->args;
    if (args->sock >= 0) {
        close(args->sock);
        args->sock = -1;
    }
    return true;
}

static size_t custom_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err) {
    custom_transport_args_t *args = (custom_transport_args_t *)transport->args;
    ssize_t sent = sendto(args->sock, buf, len, 0, (struct sockaddr *)&args->agent_addr, sizeof(args->agent_addr));
    *err = (sent < 0) ? 1 : 0;
    return (sent >= 0) ? (size_t)sent : 0;
}

static size_t custom_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout_ms, uint8_t *err) {
    custom_transport_args_t *args = (custom_transport_args_t *)transport->args;
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };
    setsockopt(args->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ssize_t received = recv(args->sock, buf, len, 0);
    *err = (received < 0) ? 1 : 0;
    return (received >= 0) ? (size_t)received : 0;
}

/* =========================================================================
 * Standard ESP-IDF Wi-Fi Initialization
 * ========================================================================= */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected. Reconnecting...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected to AP. IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

/* =========================================================================
 * Application Logic
 * ========================================================================= */
static void init_covariance_matrices(nav_msgs__msg__Odometry *odom, sensor_msgs__msg__Imu *imu) {
    static char odom_frame_id_storage[16] = "odom";
    static char odom_child_frame_id_storage[16] = "base_link";
    static char imu_frame_id_storage[16] = "base_link";

    odom->header.frame_id.data = odom_frame_id_storage;
    odom->header.frame_id.size = strlen(odom_frame_id_storage);
    odom->header.frame_id.capacity = sizeof(odom_frame_id_storage);

    odom->child_frame_id.data = odom_child_frame_id_storage;
    odom->child_frame_id.size = strlen(odom_child_frame_id_storage);
    odom->child_frame_id.capacity = sizeof(odom_child_frame_id_storage);

    imu->header.frame_id.data = imu_frame_id_storage;
    imu->header.frame_id.size = strlen(imu_frame_id_storage);
    imu->header.frame_id.capacity = sizeof(imu_frame_id_storage);

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

    // 1. Initialize options and set Agent UDP target directly
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    // 2. Initialize support
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    const int timeout_ms = 1000;
    while (rmw_uros_ping_agent(timeout_ms, 1) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Waiting for micro-ROS agent handshake at %s:%s...", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
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

    ESP_LOGI(TAG, "micro-ROS streaming simulated spin...");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 Hz
    const float dt = 0.05f;
    const float spin_speed = 1.0f;

    float yaw = 0.0f;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        yaw += spin_speed * dt;
        if (yaw > 2.0f * (float)M_PI) {
            yaw -= 2.0f * (float)M_PI;
        }

        float qz = sinf(yaw * 0.5f);
        float qw = cosf(yaw * 0.5f);

        int64_t time_ns = rmw_uros_epoch_nanos();
        int32_t sec = time_ns / 1000000000;
        uint32_t nanosec = time_ns % 1000000000;

        // Odometry
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

        // IMU
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

    // Initialize native Wi-Fi
    wifi_init_sta();

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