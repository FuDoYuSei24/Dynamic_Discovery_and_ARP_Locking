// station/main.c - 修复设备发现
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
#define DEVICE_ID        "Robot-2"  // Robot-1 或 Robot-2
#define DISCOVERY_PERIOD_MS 30000

static const char *TAG = "STATION";
static esp_netif_t *sta_netif = NULL;
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// 全局状态
static bool discovery_phase = true;
static bool arp_locked = false;
static uint32_t system_start_time = 0;

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

// ================== 修复的广播发送函数 ==================
void broadcast_discovery(void)
{
    if (!discovery_phase) {
        return;
    }
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "创建广播socket失败");
        return;
    }
    
    // 允许广播
    int broadcast_enable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        ESP_LOGE(TAG, "设置广播选项失败");
        close(sock);
        return;
    }
    
    // 使用定向广播地址（192.168.4.255）而不是全局广播
    struct sockaddr_in broadcast_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DISCOVERY_PORT),
        .sin_addr.s_addr = inet_addr("192.168.4.255")  // 定向广播
    };
    
    // 如果inet_addr失败，使用另一种方式设置广播地址
    if (broadcast_addr.sin_addr.s_addr == INADDR_NONE) {
        broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    }
    
    char msg[256];
    snprintf(msg, sizeof(msg),
             "DISCOVERY:ID=%s:IP=" IPSTR ":MAC=%02x:%02x:%02x:%02x:%02x:%02x",
             DEVICE_ID,
             IP2STR(&s_local_ip),
             s_local_mac[0], s_local_mac[1], s_local_mac[2],
             s_local_mac[3], s_local_mac[4], s_local_mac[5]);
    
    ESP_LOGD(TAG, "发送广播: %s", msg);
    
    int sent = sendto(sock, msg, strlen(msg), 0, 
                     (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
    
    if (sent < 0) {
        ESP_LOGE(TAG, "广播发送失败 (errno=%d)", errno);
    } else {
        ESP_LOGD(TAG, "广播发送成功，%d字节", sent);
    }
    
    close(sock);
}

// ================== 修复的广播接收任务 ==================
void discovery_receiver_task(void *pvParameters)
{
    ESP_LOGI(TAG, "广播接收任务启动");
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "创建接收socket失败");
        vTaskDelete(NULL);
        return;
    }
    
    // 允许地址重用
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DISCOVERY_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };
    
    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "绑定端口失败 (errno=%d)", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "已绑定到端口 %d", DISCOVERY_PORT);
    
    char buffer[256];
    
    while (1) {
        struct sockaddr_in src_addr;
        socklen_t addr_len = sizeof(src_addr);
        
        int len = recvfrom(sock, buffer, sizeof(buffer)-1, 0,
                          (struct sockaddr *)&src_addr, &addr_len);
        
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGD(TAG, "收到广播: %s (来自 %s:%d)", 
                    buffer, inet_ntoa(src_addr.sin_addr), ntohs(src_addr.sin_port));
            
            // 如果在锁定阶段，忽略发现消息
            if (arp_locked) {
                continue;
            }
            
            // 解析发现消息
            if (strncmp(buffer, "DISCOVERY:ID=", 13) == 0) {
                char device_id[32];
                char ip_str[16];
                uint8_t mac[6];
                
                // 解析格式: DISCOVERY:ID=xxx:IP=xxx.xxx.xxx.xxx:MAC=xx:xx:xx:xx:xx:xx
                if (sscanf(buffer, "DISCOVERY:ID=%31[^:]:IP=%15[^:]:MAC=%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                          device_id, ip_str,
                          &mac[0], &mac[1], &mac[2],
                          &mac[3], &mac[4], &mac[5]) == 8) {
                    
                    ESP_LOGD(TAG, "解析成功: ID=%s, IP=%s", device_id, ip_str);
                    
                    // 跳过自己的广播
                    if (strcmp(device_id, DEVICE_ID) == 0) {
                        ESP_LOGD(TAG, "忽略自己的广播");
                        continue;
                    }
                    
                    ip4_addr_t peer_ip;
                    if (ip4addr_aton(ip_str, &peer_ip) == 0) {
                        ESP_LOGE(TAG, "IP地址解析失败: %s", ip_str);
                        continue;
                    }
                    
                    xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
                    
                    bool found = false;
                    for (int i = 0; i < peer_count; i++) {
                        if (peer_table[i].ip.addr == peer_ip.addr || 
                            strcmp(peer_table[i].device_id, device_id) == 0) {
                            // 更新现有设备
                            memcpy(peer_table[i].mac, mac, 6);
                            strcpy(peer_table[i].device_id, device_id);
                            peer_table[i].last_seen = xTaskGetTickCount();
                            peer_table[i].ip = peer_ip;
                            found = true;
                            
                            ESP_LOGI(TAG, "更新设备: %s (" IPSTR ")", 
                                    device_id, IP2STR(&peer_ip));
                            break;
                        }
                    }
                    
                    if (!found && peer_count < MAX_PEERS) {
                        // 添加新设备
                        peer_table[peer_count].ip = peer_ip;
                        memcpy(peer_table[peer_count].mac, mac, 6);
                        strcpy(peer_table[peer_count].device_id, device_id);
                        peer_table[peer_count].last_seen = xTaskGetTickCount();
                        peer_table[peer_count].arp_injected = false;
                        peer_count++;
                        
                        ESP_LOGI(TAG, "📱 发现新设备: %s (IP:" IPSTR " MAC:%02x:%02x:%02x:%02x:%02x:%02x)", 
                                device_id, IP2STR(&peer_ip),
                                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                    }
                    
                    xSemaphoreGive(peer_table_mutex);
                } else {
                    ESP_LOGW(TAG, "广播消息格式错误: %s", buffer);
                }
            } else {
                ESP_LOGD(TAG, "非发现消息: %s", buffer);
            }
        } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGE(TAG, "接收错误 (errno=%d)", errno);
        }
    }
}

// ================== 增强的ARP管理 ==================
void manage_arp_entries(void)
{
    xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
    
    for (int i = 0; i < peer_count; i++) {
        if (!peer_table[i].arp_injected) {
            if (arp_locked) {
                // 锁定阶段发现新设备，警告但不注入
                peer_table[i].arp_injected = true;
                ESP_LOGW(TAG, "⚠️ 锁定阶段发现新设备 %s，跳过ARP注入", 
                        peer_table[i].device_id);
                continue;
            }
            
            // 发现阶段：注入ARP
            struct eth_addr peer_mac;
            memcpy(peer_mac.addr, peer_table[i].mac, 6);
            
            // 先移除可能存在的旧条目
            etharp_remove_static_entry(&peer_table[i].ip);
            
            // 添加静态ARP条目
            err_t err = etharp_add_static_entry(&peer_table[i].ip, &peer_mac);
            if (err == ERR_OK) {
                peer_table[i].arp_injected = true;
                ESP_LOGI(TAG, "✅ ARP注入: %s (" IPSTR ") → %02x:%02x:%02x:%02x:%02x:%02x", 
                        peer_table[i].device_id, IP2STR(&peer_table[i].ip),
                        peer_table[i].mac[0], peer_table[i].mac[1], peer_table[i].mac[2],
                        peer_table[i].mac[3], peer_table[i].mac[4], peer_table[i].mac[5]);
            } else {
                ESP_LOGE(TAG, "❌ ARP注入失败: %s (err=%d)", 
                        peer_table[i].device_id, err);
            }
        }
    }
    
    xSemaphoreGive(peer_table_mutex);
}

// ================== 修复的发送数据函数 ==================
void send_to_all_peers(void)
{
    if (peer_count == 0) {
        ESP_LOGW(TAG, "没有发现任何设备，无法发送数据");
        return;
    }
    
    xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
    
    for (int i = 0; i < peer_count; i++) {
        if (!peer_table[i].arp_injected) {
            ESP_LOGW(TAG, "跳过设备 %s (ARP未注入)", peer_table[i].device_id);
            continue;
        }
        
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            ESP_LOGE(TAG, "创建socket失败 (errno=%d)", errno);
            continue;
        }
        
        // 设置发送超时
        struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        
        struct sockaddr_in dest_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(12345),
            .sin_addr.s_addr = peer_table[i].ip.addr
        };
        
        char msg[128];
        if (arp_locked) {
            snprintf(msg, sizeof(msg), "🔒 锁定阶段: %s → %s (时间: %lu)", 
                    DEVICE_ID, peer_table[i].device_id, 
                    xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
        } else {
            snprintf(msg, sizeof(msg), "🔍 发现阶段: %s → %s", 
                    DEVICE_ID, peer_table[i].device_id);
        }
        
        int sent = sendto(sock, msg, strlen(msg), 0, 
                         (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        
        if (sent < 0) {
            if (errno == EHOSTUNREACH) {
                ESP_LOGE(TAG, "发送失败: 主机不可达 (目标: %s " IPSTR ")", 
                        peer_table[i].device_id, IP2STR(&peer_table[i].ip));
            } else {
                ESP_LOGE(TAG, "发送失败 (errno=%d)", errno);
            }
        } else {
            ESP_LOGI(TAG, "✅ 发送成功: %s → %s (%d字节)", 
                    DEVICE_ID, peer_table[i].device_id, sent);
        }
        
        close(sock);
    }
    
    xSemaphoreGive(peer_table_mutex);
}

// ================== 时间管理 ==================
void check_system_phase(void)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now - system_start_time;
    
    if (discovery_phase && elapsed >= DISCOVERY_PERIOD_MS) {
        discovery_phase = false;
        arp_locked = true;
        
        ESP_LOGI(TAG, "🚀 发现阶段结束，进入ARP锁定阶段");
        ESP_LOGI(TAG, "🔒 ARP表已锁定，共发现 %d 个设备", peer_count);
        
        xSemaphoreTake(peer_table_mutex, portMAX_DELAY);
        for (int i = 0; i < peer_count; i++) {
            ESP_LOGI(TAG, "  设备 %d: %s (" IPSTR ")", 
                    i+1, peer_table[i].device_id, IP2STR(&peer_table[i].ip));
        }
        xSemaphoreGive(peer_table_mutex);
    }
}

// ================== 防止ARP请求 ==================
void suppress_arp_requests(void)
{
    // 禁用ARP请求发送
    struct netif *netif = netif_default;
    if (netif) {
        netif->flags &= ~NETIF_FLAG_ETHARP;
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

// ================== 主函数 - 增强调试 ==================
void app_main(void)
{
    ESP_LOGI(TAG, "========== 设备 %s 启动 ==========", DEVICE_ID);
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NEW_VERSION_FOUND || ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 网络初始化
    esp_netif_init();
    esp_event_loop_create_default();
    
    // 记录启动时间
    system_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // 初始化设备表
    peer_table_mutex = xSemaphoreCreateMutex();
    
    // 连接WiFi
    station_init();
    ESP_LOGI(TAG, "设备 %s 启动，发现阶段30秒", DEVICE_ID);
    
    // 启动广播接收任务
    ESP_LOGI(TAG, "启动广播接收任务...");
    xTaskCreate(discovery_receiver_task, "discovery_rcv", 4096, NULL, 5, NULL);
    
    // 等待网络稳定
    ESP_LOGI(TAG, "等待网络稳定 (3秒)...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 主循环
    int broadcast_counter = 0;
    int arp_manage_counter = 0;
    bool arp_suppressed = false;
    
    ESP_LOGI(TAG, "开始主循环...");
    
    while (1) {
        // 检查系统阶段
        check_system_phase();
        
        // 发现阶段：每秒发送一次广播（增加频率）
        if (discovery_phase && broadcast_counter++ >= 1) {
            broadcast_discovery();
            broadcast_counter = 0;
        }
        
        // 管理ARP条目
        if (arp_manage_counter++ >= (arp_locked ? 10 : 2)) {
            manage_arp_entries();
            arp_manage_counter = 0;
        }
        
        // 进入锁定阶段后，执行ARP抑制
        if (arp_locked && !arp_suppressed) {
            suppress_arp_requests();
            arp_suppressed = true;
            ESP_LOGI(TAG, "🔐 系统已锁定，开始正常通信（无ARP）");
        }
        
        // 发送数据
        static int data_counter = 0;
        int send_interval = arp_locked ? 2 : 3;  // 锁定阶段更频繁发送
        if (data_counter++ >= send_interval) {
            send_to_all_peers();
            data_counter = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 显示状态信息
        static int status_counter = 0;
        if (status_counter++ >= 5) {  // 每5秒显示一次
            ESP_LOGI(TAG, "状态: %s阶段 | 发现设备数: %d | ARP锁定: %s",
                    discovery_phase ? "发现" : "锁定",
                    peer_count,
                    arp_locked ? "是" : "否");
            status_counter = 0;
        }
    }
}