// station/main.c - 带时间窗口的动态发现+ARP锁定
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/etharp.h"
#include "lwip/ip4_addr.h"

#define WIFI_SSID        "RobotNet"
#define WIFI_PASS        "12345678"
#define DISCOVERY_PORT   8888
#define DEVICE_ID        "Robot-2"  // 每个设备烧录不同的ID
#define DISCOVERY_PERIOD_MS 30000   // 发现阶段持续时间：30秒

static const char *TAG = "STATION";
static esp_netif_t *sta_netif = NULL;
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// 全局状态
static bool discovery_phase = true;     // 是否在发现阶段
static bool arp_locked = false;         // ARP表是否已锁定
static uint32_t system_start_time = 0;  // 系统启动时间

static ip4_addr_t s_local_ip;
static uint8_t s_local_mac[6];

// 设备表结构
typedef struct {
    ip4_addr_t ip;
    uint8_t mac[6];
    char device_id[32];
    uint32_t last_seen;
    bool arp_injected;
} peer_device_t;

#define MAX_PEERS 10
static peer_device_t peer_table[MAX_PEERS];
static int peer_count = 0;
static SemaphoreHandle_t peer_table_mutex;

// ================== 时间管理 ==================
void check_system_phase(void)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now - system_start_time;
    
    if (discovery_phase && elapsed >= DISCOVERY_PERIOD_MS) {
        // 进入锁定阶段
        discovery_phase = false;
        arp_locked = true;
        ESP_LOGI(TAG, "🚀 发现阶段结束，进入ARP锁定阶段");
        ESP_LOGI(TAG, "🔒 ARP表已锁定，共发现 %d 个设备", peer_count);
        
        // 打印所有已知设备
        xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
        for (int i = 0; i < peer_count; i++) {
            ESP_LOGI(TAG, "  设备 %d: %s (" IPSTR ")", 
                    i+1, peer_table[i].device_id, IP2STR(&peer_table[i].ip));
        }
        xSemaphoreGive(peer_table_mutex);
    }
}

// ================== UDP 广播发送 ==================
void broadcast_discovery(void)
{
    if (!discovery_phase) {
        return;  // 发现阶段结束，不再发送广播
    }
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) return;
    
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    
    struct sockaddr_in broadcast_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DISCOVERY_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST)
    };
    
    char msg[256];
    snprintf(msg, sizeof(msg),
             "DISCOVERY:ID=%s:IP=" IPSTR ":MAC=%02x:%02x:%02x:%02x:%02x:%02x",
             DEVICE_ID,
             IP2STR(&s_local_ip),
             s_local_mac[0], s_local_mac[1], s_local_mac[2],
             s_local_mac[3], s_local_mac[4], s_local_mac[5]);
    
    sendto(sock, msg, strlen(msg), 0, 
           (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
    
    close(sock);
}

// ================== UDP 广播接收 ==================
void discovery_receiver_task(void *pvParameters)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Discovery receiver socket failed");
        vTaskDelete(NULL);
        return;
    }
    
    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DISCOVERY_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };
    
    bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
    
    char buffer[256];
    
    while (1) {
        struct sockaddr_in src_addr;
        socklen_t addr_len = sizeof(src_addr);
        
        int len = recvfrom(sock, buffer, sizeof(buffer)-1, 0,
                          (struct sockaddr *)&src_addr, &addr_len);
        
        if (len > 0) {
            buffer[len] = '\0';
            
            // 如果在锁定阶段，忽略发现消息
            if (arp_locked) {
                continue;
            }
            
            if (strncmp(buffer, "DISCOVERY:ID=", 13) == 0) {
                char device_id[32];
                char ip_str[16];
                uint8_t mac[6];
                
                if (sscanf(buffer, "DISCOVERY:ID=%31[^:]:IP=%15[^:]:MAC=%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                          device_id, ip_str,
                          &mac[0], &mac[1], &mac[2],
                          &mac[3], &mac[4], &mac[5]) == 8) {
                    
                    if (strcmp(device_id, DEVICE_ID) == 0) {
                        continue;
                    }
                    
                    ip4_addr_t peer_ip;
                    ip4addr_aton(ip_str, &peer_ip);
                    
                    xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
                    
                    bool found = false;
                    for (int i = 0; i < peer_count; i++) {
                        if (peer_table[i].ip.addr == peer_ip.addr) {
                            memcpy(peer_table[i].mac, mac, 6);
                            strcpy(peer_table[i].device_id, device_id);
                            peer_table[i].last_seen = xTaskGetTickCount();
                            found = true;
                            break;
                        }
                    }
                    
                    if (!found && peer_count < MAX_PEERS) {
                        peer_table[peer_count].ip = peer_ip;
                        memcpy(peer_table[peer_count].mac, mac, 6);
                        strcpy(peer_table[peer_count].device_id, device_id);
                        peer_table[peer_count].last_seen = xTaskGetTickCount();
                        peer_table[peer_count].arp_injected = false;
                        peer_count++;
                        
                        ESP_LOGI(TAG, "📱 发现新设备: %s (IP:" IPSTR ")", 
                                device_id, IP2STR(&peer_ip));
                    }
                    
                    xSemaphoreGive(peer_table_mutex);
                }
            }
        }
    }
}

// ================== ARP 管理 ==================
void manage_arp_entries(void)
{
    xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
    
    // 如果ARP已锁定，只维护现有条目，不添加新条目
    for (int i = 0; i < peer_count; i++) {
        if (!peer_table[i].arp_injected) {
            // 如果是锁定阶段，直接标记为已注入（不再实际注入）
            if (arp_locked) {
                peer_table[i].arp_injected = true;
                ESP_LOGW(TAG, "⚠️  锁定阶段发现新设备 %s，但不会注入ARP", 
                        peer_table[i].device_id);
                continue;
            }
            
            // 发现阶段：正常注入ARP
            struct eth_addr peer_mac;
            memcpy(peer_mac.addr, peer_table[i].mac, 6);
            
            err_t err = etharp_add_static_entry(&peer_table[i].ip, &peer_mac);
            if (err == ERR_OK) {
                peer_table[i].arp_injected = true;
                ESP_LOGI(TAG, "✅ ARP注入: %s (" IPSTR ")", 
                        peer_table[i].device_id, IP2STR(&peer_table[i].ip));
            }
        }
    }
    
    xSemaphoreGive(peer_table_mutex);
}

// ================== 发送数据（在锁定阶段使用静态ARP）==================
void send_to_all_peers(void)
{
    xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
    
    for (int i = 0; i < peer_count; i++) {
        // 检查设备是否支持通信
        if (!peer_table[i].arp_injected) {
            continue;
        }
        
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) continue;
        
        struct sockaddr_in dest_addr = {0};
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(12345);
        dest_addr.sin_addr.s_addr = peer_table[i].ip.addr;
        
        char msg[128];
        if (arp_locked) {
            snprintf(msg, sizeof(msg), "LOCKED: %s → %s (无ARP)", 
                    DEVICE_ID, peer_table[i].device_id);
        } else {
            snprintf(msg, sizeof(msg), "DISCOVERY: %s → %s", 
                    DEVICE_ID, peer_table[i].device_id);
        }
        
        sendto(sock, msg, strlen(msg), 0, 
               (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        
        close(sock);
    }
    
    xSemaphoreGive(peer_table_mutex);
}

// ================== 获取本地MAC地址 ==================
void get_local_mac(void)
{
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, s_local_mac);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "本地MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                s_local_mac[0], s_local_mac[1], s_local_mac[2],
                s_local_mac[3], s_local_mac[4], s_local_mac[5]);
    }
}

// ================== Wi-Fi 事件处理 ==================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "断开连接，重连中...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        s_local_ip.addr = event->ip_info.ip.addr;
        ESP_LOGI(TAG, "获得IP: " IPSTR, IP2STR(&s_local_ip));
        
        get_local_mac();
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ================== Station 初始化 ==================
void station_init(void)
{
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    s_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

// ================== 防止ARP请求的终极方案 ==================
void suppress_arp_requests(void)
{
    // 修改lwIP配置，禁用ARP请求
    extern struct netif *netif_default;
    
    if (netif_default) {
        // 设置接口不发送ARP请求
        netif_default->flags &= ~NETIF_FLAG_ETHARP;
        ESP_LOGI(TAG, "🛑 已禁用ARP请求发送");
    }
    
    // 确保所有已知设备都有ARP条目
    xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
    for (int i = 0; i < peer_count; i++) {
        if (peer_table[i].arp_injected) {
            struct eth_addr peer_mac;
            memcpy(peer_mac.addr, peer_table[i].mac, 6);
            etharp_add_static_entry(&peer_table[i].ip, &peer_mac);
        }
    }
    xSemaphoreGive(peer_table_mutex);
}

// ================== 主函数 ==================
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NEW_VERSION_FOUND || ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    esp_netif_init();
    esp_event_loop_create_default();
    
    // 记录系统启动时间
    system_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // 初始化设备表
    peer_table_mutex = xSemaphoreCreateMutex();
    
    station_init();
    ESP_LOGI(TAG, "设备 %s 启动，发现阶段30秒", DEVICE_ID);
    
    // 创建发现接收任务
    xTaskCreate(discovery_receiver_task, "discovery_rcv", 4096, NULL, 5, NULL);
    
    // 等待网络稳定
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 主循环
    int broadcast_counter = 0;
    int arp_manage_counter = 0;
    bool arp_suppressed = false;
    
    while (1) {
        // 检查系统阶段
        check_system_phase();
        
        // 发现阶段：每3秒发送一次广播
        if (discovery_phase && broadcast_counter++ >= 3) {
            broadcast_discovery();
            broadcast_counter = 0;
        }
        
        // 管理ARP条目（发现阶段每2秒一次，锁定阶段每10秒一次）
        if (arp_manage_counter++ >= (arp_locked ? 10 : 2)) {
            manage_arp_entries();
            arp_manage_counter = 0;
        }
        
        // 进入锁定阶段后，执行一次ARP抑制
        if (arp_locked && !arp_suppressed) {
            suppress_arp_requests();
            arp_suppressed = true;
            ESP_LOGI(TAG, "🔐 系统已锁定，开始正常通信（无ARP）");
        }
        
        // 发送数据（发现阶段每5秒，锁定阶段每3秒）
        static int data_counter = 0;
        int send_interval = arp_locked ? 3 : 5;
        if (data_counter++ >= send_interval) {
            send_to_all_peers();
            data_counter = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 显示状态信息
        static int status_counter = 0;
        if (status_counter++ >= 10) {
            ESP_LOGI(TAG, "状态: %s阶段 | 发现设备数: %d | ARP锁定: %s",
                    discovery_phase ? "发现" : "锁定",
                    peer_count,
                    arp_locked ? "是" : "否");
            status_counter = 0;
        }
    }
}