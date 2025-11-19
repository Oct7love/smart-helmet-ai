#ifndef __FAN_CONTROLLER_H__
#define __FAN_CONTROLLER_H__

#include "mcp_server.h"
#include <esp_log.h>

class FanController {
private:
    bool power_ = false;
    gpio_num_t gpio_num_;
    static constexpr const char* TAG = "FanController";

public:
    FanController(gpio_num_t gpio_num) : gpio_num_(gpio_num) {
        ESP_LOGI(TAG, "Initializing FanController on GPIO %d", gpio_num_);
        gpio_config_t config = {
            .pin_bit_mask = (1ULL << gpio_num_),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&config));
        gpio_set_level(gpio_num_, 0);
        ESP_LOGI(TAG, "FanController initialized, GPIO %d set to LOW", gpio_num_);

        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.fan.get_state", "Get the power state of the fan", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "get_state called, power=%d", power_);
            return power_ ? "{\"power\": true}" : "{\"power\": false}";
        });

        mcp_server.AddTool("self.fan.turn_on", "Turn on the fan", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "turn_on called, setting GPIO %d to HIGH", gpio_num_);
            power_ = true;
            gpio_set_level(gpio_num_, 1);
            return true;
        });

        mcp_server.AddTool("self.fan.turn_off", "Turn off the fan", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "turn_off called, setting GPIO %d to LOW", gpio_num_);
            power_ = false;
            gpio_set_level(gpio_num_, 0);
            return true;
        });
    }
};

#endif // __FAN_CONTROLLER_H__
