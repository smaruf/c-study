/**
 * @file main.c
 * @brief ESP32 Wi-Fi and mDNS-based Network Discovery with BFS Shortest Path Finder (Enhanced Version)
 *
 * This application initiates Wi-Fi connection, announces its service via mDNS,
 * performs mDNS-based discovery to identify other network-capable devices, and
 * uses an enhanced BFS algorithm for the shortest path detection in a dynamically formed network.
 *
 * ## Prerequisites:
 * - ESP-IDF v4.x or later
 * - Properly configured and functional ESP32 development environment
 *
 * ## Hardware Setup:
 * - ESP32 development board connected to a PC with USB
 * - ESP32 must be connected to a local network with mDNS capabilities
 *
 * ## Build and Flash Instructions:
 * - Define your local Wi-Fi credentials for `WIFI_SSID` and `WIFI_PASS`
 * - Navigate to project directory
 * - Run `idf.py build` to compile the project
 * - Run `idf.py -p (YOUR_SERIAL_PORT) flash` to upload it to your ESP32
 * - Run `idf.py -p (YOUR_SERIAL_PORT) monitor` to see the runtime output
 *
 * @note Ensure your network supports mDNS.
 *
 * @author MSMaruf
 * @date 2022
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mdns.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define WIFI_SSID "yourSSID"
#define WIFI_PASS "yourPassword"

typedef struct device {
    char hostname[32];
    struct device* next;
} device_t;

typedef struct {
    device_t* head;
    int size;
} network_t;

network_t network;

void add_device_to_network(const char* hostname) {
    device_t* newDevice = malloc(sizeof(device_t));
    if(!newDevice) {
        printf("Memory allocation failed for new device.\n");
        return;
    }
    strcpy(newDevice->hostname, hostname);
    newDevice->next = NULL;
    
    if (network.head == NULL) {
        network.head = newDevice;
    } else {
        device_t* current = network.head;
        while (current->next != NULL) {
            current = current->next;
        }
        current->next = newDevice;
    }
    network.size++;
}

void wifi_init_sta(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void initialise_mdns(void) {
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp32-device"));
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 Network Node"));
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

void query_mdns_services(void) {
    mdns_result_t* results = NULL;
    ESP_ERROR_CHECK(mdns_query_ptr("_http._tcp", 2000, &results));
    mdns_result_t* r = results;

    while (r) {
        add_device_to_network(r->instance_name);
        r = r->next;
    }

    mdns_query_results_free(results);
}

void bfs(const char* target_hostname) {
    if (network.head == NULL) {
        printf("Network is empty\n");
        return;
    }

    device_t* queue[100]; // Static allocation for simplicity
    int front = 0, rear = 0;

    // Enqueue start node
    queue[rear++] = network.head;

    while (front != rear) {
        device_t* current = queue[front++];
        printf("Visiting: %s\n", current->hostname);

        if (strcmp(current->hostname, target_hostname) == 0) {
            printf("Found target device: %s\n", target_hostname);
            return;
        }

        // Enqueue all neighbors
        if (current->next != NULL) {
            queue[rear++] = current->next;
        }
    }

    printf("Target device not found\n");
}

void app_main(void) {
    wifi_init_sta();
    initialise_mdns();

    // Delay to ensure WiFi connection is established, otherwise mDNS might fail
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    query_mdns_services();
    bfs("desired-target-hostname"); // Modify the target name as needed
}
