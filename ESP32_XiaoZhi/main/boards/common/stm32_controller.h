#ifndef __STM32_CONTROLLER_H__
#define __STM32_CONTROLLER_H__

#include "mcp_server.h"
#include "display.h"
#include "board.h"
#include "application.h"
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

#define STM32_UART_NUM UART_NUM_1
#define STM32_TXD_PIN  (GPIO_NUM_17)
#define STM32_RXD_PIN  (GPIO_NUM_18)
#define STM32_UART_BAUD_RATE 115200
#define BUF_SIZE (1024)

class STM32Controller {
private:
    bool heart_rate_active_ = false;
    bool temperature_active_ = false;
    bool mpu6050_active_ = false;
    bool gps_active_ = false;
    bool mq2_active_ = false;

    int latest_heart_rate_ = 0;
    int latest_spo2_ = 0;
    float latest_temperature_ = 0.0f;
    int latest_humidity_ = 0;           // ← 新添加
    float latest_pitch_ = 0.0f;
    float latest_roll_ = 0.0f;
    float latest_yaw_ = 0.0f;
    float latest_mq2_ppm_ = 0.0f;      // ← 新添加的第35行
    int latest_mq2_alarm_ = 0;          // ← 新添加的第36行

        // GPS数据
      float latest_latitude_ = 0.0f;
      float latest_longitude_ = 0.0f;
      int latest_satellites_ = 0;


    static constexpr const char* TAG = "STM32Controller";

    void InitializeUART() {
        uart_config_t uart_config = {
            .baud_rate = STM32_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(uart_param_config(STM32_UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(STM32_UART_NUM, STM32_TXD_PIN, STM32_RXD_PIN,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_driver_install(STM32_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

        ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d",
                 STM32_TXD_PIN, STM32_RXD_PIN, STM32_UART_BAUD_RATE);

        xTaskCreate(uart_rx_task_static, "stm32_uart_rx", 4096, this, 10, NULL);
    }

    static void uart_rx_task_static(void* arg) {
        STM32Controller* controller = (STM32Controller*)arg;
        controller->uart_rx_task();
    }

    void uart_rx_task() {
        uint8_t data[BUF_SIZE];
        while (1) {
            int len = uart_read_bytes(STM32_UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
            if (len > 0) {
                data[len] = 0;
                ProcessResponse((char*)data);
            }
        }
    }

    void ProcessResponse(const char* response) {
        ESP_LOGI(TAG, "收到STM32数据: %s", response);

        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        char notify_msg[128];

        if (strstr(response, "{DATA:HR:")) {
            int hr = 0, spo2 = 0;
            if (sscanf(response, "{DATA:HR:%d,SPO2:%d}", &hr, &spo2) == 2) {
                latest_heart_rate_ = hr;
                latest_spo2_ = spo2;
                ESP_LOGI(TAG, "心率: %d bpm, 血氧: %d%%", hr, spo2);
                snprintf(notify_msg, sizeof(notify_msg), "心率:%d 血氧:%d%%", hr, spo2);
                if (display) display->ShowNotification(notify_msg);
            }
        }
        else if (strstr(response, "{DATA:TEMP:")) {
              int temp = 0, humi = 0;
              if (sscanf(response, "{DATA:TEMP:%d,%d}", &temp, &humi) == 2) {
                  latest_temperature_ = (float)temp;
                  latest_humidity_ = humi;
                  ESP_LOGI(TAG, "温度: %d°C, 湿度: %d%%", temp, humi);
                  snprintf(notify_msg, sizeof(notify_msg), "温度:%d°C 湿度:%d%%", temp, humi);
                  if (display) display->ShowNotification(notify_msg);
              }
          }
        else if (strstr(response, "{DATA:MPU:")) {
            float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
            if (sscanf(response, "{DATA:MPU:%f,%f,%f}", &pitch, &roll, &yaw) == 3) {
                latest_pitch_ = pitch;
                latest_roll_ = roll;
                latest_yaw_ = yaw;
                ESP_LOGI(TAG, "姿态: Pitch=%.1f° Roll=%.1f° Yaw=%.1f°", pitch, roll, yaw);
                if (display) display->ShowNotification("姿态数据已更新");
            }
        }
         else if (strstr(response, "{DATA:MQ2:")) {    // ← 从这里开始新添加
              float ppm = 0.0f;
              int alarm = 0;
              if (sscanf(response, "{DATA:MQ2:%f,%d}", &ppm, &alarm) == 2) {
                  latest_mq2_ppm_ = ppm;
                  latest_mq2_alarm_ = alarm;
                  ESP_LOGI(TAG, "烟雾: PPM=%.1f, 报警级别=%d", ppm, alarm);

                  // 显示数据
                  char notify_msg[128];
                  const char* level_str[] = {"安全", "低", "中", "高"};
                  snprintf(notify_msg, sizeof(notify_msg), "烟雾:%.1fppm [%s]",
                          ppm, alarm < 4 ? level_str[alarm] : "危险");
                  if (display) display->ShowNotification(notify_msg);
              }
          }  
           else if (strstr(response, "{DATA:GPS:")) {
              float lat = 0.0f, lon = 0.0f;
              int satellites = 0;
              if (sscanf(response, "{DATA:GPS:%f,%f,%d}", &lat, &lon, &satellites) == 3) {
                  latest_latitude_ = lat;
                  latest_longitude_ = lon;
                  latest_satellites_ = satellites;
                  ESP_LOGI(TAG, "GPS: 纬度=%.6f, 经度=%.6f, 卫星=%d", lat, lon, satellites);
                  snprintf(notify_msg, sizeof(notify_msg), "GPS:%.5f,%.5f", lat, lon);
                  if (display) display->ShowNotification(notify_msg);
              }
          }                                
        else if (strstr(response, "{STATUS:")) {
            if (strstr(response, "OK")) {
                ESP_LOGI(TAG, "命令执行成功");
                if (display) display->ShowNotification("STM32: OK");
            } else if (strstr(response, "ERROR")) {
                ESP_LOGW(TAG, "命令执行失败");
                if (display) display->ShowNotification("STM32: ERROR");
            }
        }
        else if (strstr(response, "{EVENT:FALL}")) {
            ESP_LOGW(TAG, "⚠️ 检测到摔倒事件！");

            auto& board = Board::GetInstance();
            auto display = board.GetDisplay();
            if (display) {
                display->ShowNotification("⚠️ 摔倒警告！");
            }

            // 播放警告音效
            auto& app = Application::GetInstance();
            app.PlaySound("alert");

            ESP_LOGW(TAG, "警告：检测到摔倒，请注意安全！");
        }
        else if (strstr(response, "{EVENT:COLLISION}")) {
            ESP_LOGW(TAG, "⚠️ 检测到碰撞事件！");

            auto& board = Board::GetInstance();
            auto display = board.GetDisplay();
            if (display) {
                display->ShowNotification("⚠️ 碰撞警告！");
            }

            // 播放警告音效
            auto& app = Application::GetInstance();
            app.PlaySound("alert");

            ESP_LOGW(TAG, "警告：检测到碰撞，请小心！");
        }else if (strstr(response, "{EVENT:SMOKE}")) {    // ← 从这里开始添加
              ESP_LOGW(TAG, "⚠️ 检测到烟雾事件！");

              auto& board = Board::GetInstance();
              auto display = board.GetDisplay();
              if (display) {
                  display->ShowNotification("⚠️ 烟雾警告！");
              }

              // 播放警告音效
              auto& app = Application::GetInstance();
              app.PlaySound("alert");

              ESP_LOGW(TAG, "警告：检测到烟雾，请远离火源！");
          }      

    }

    void SendCommand(const char* cmd) {
        int len = strlen(cmd);
        int written = uart_write_bytes(STM32_UART_NUM, cmd, len);
        if (written == len) {
            ESP_LOGI(TAG, "发送命令: %s", cmd);
        } else {
            ESP_LOGE(TAG, "发送命令失败");
        }
    }

public:
    STM32Controller() {
        ESP_LOGI(TAG, "Initializing STM32Controller");
        InitializeUART();

        auto& mcp_server = McpServer::GetInstance();

        // ============ 心率检测 ============
        mcp_server.AddTool("self.stm32.heart_rate.get_state",
            "Get the current heart rate detection state and values (heart rate in bpm and SpO2 in %)",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                char json[256];
                snprintf(json, sizeof(json),
                    "{\"active\": %s, \"heart_rate\": %d, \"spo2\": %d}",
                    heart_rate_active_ ? "true" : "false",
                    latest_heart_rate_,
                    latest_spo2_);
                return std::string(json);
            });

        mcp_server.AddTool("self.stm32.heart_rate.start",
            "Start heart rate and blood oxygen (SpO2) monitoring",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                heart_rate_active_ = true;
                SendCommand("{CMD:HR_START}\r\n");
                ESP_LOGI(TAG, "心率检测已启动");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("心率检测已启动");
                return true;
            });

        mcp_server.AddTool("self.stm32.heart_rate.stop",
            "Stop heart rate and blood oxygen monitoring",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                heart_rate_active_ = false;
                SendCommand("{CMD:HR_STOP}\r\n");
                ESP_LOGI(TAG, "心率检测已停止");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("心率检测已停止");
                return true;
            });

        // ============ 温度检测 ============
       mcp_server.AddTool("self.stm32.temperature.get_state",
              "Get the current temperature and humidity values",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  char json[128];
                  snprintf(json, sizeof(json),
                      "{\"active\": %s, \"temperature\": %.1f, \"humidity\": %d}",
                      temperature_active_ ? "true" : "false",
                      latest_temperature_,
                      latest_humidity_);
                  return std::string(json);
              });

        mcp_server.AddTool("self.stm32.temperature.start",
            "Start temperature monitoring",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                temperature_active_ = true;
                SendCommand("{CMD:TEMP_START}\r\n");
                ESP_LOGI(TAG, "温度检测已启动");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("温度检测已启动");
                return true;
            });

        mcp_server.AddTool("self.stm32.temperature.stop",
            "Stop temperature monitoring",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                temperature_active_ = false;
                SendCommand("{CMD:TEMP_STOP}\r\n");
                ESP_LOGI(TAG, "温度检测已停止");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("温度检测已停止");
                return true;
            });

        // ============ 姿态检测 ============
        mcp_server.AddTool("self.stm32.mpu6050.get_state",
            "Get the current MPU6050 attitude (pitch, roll, yaw in degrees)",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                char json[256];
                snprintf(json, sizeof(json),
                    "{\"active\": %s, \"pitch\": %.1f, \"roll\": %.1f, \"yaw\": %.1f}",
                    mpu6050_active_ ? "true" : "false",
                    latest_pitch_,
                    latest_roll_,
                    latest_yaw_);
                return std::string(json);
            });

        mcp_server.AddTool("self.stm32.mpu6050.start",
            "Start MPU6050 attitude detection",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                mpu6050_active_ = true;
                SendCommand("{CMD:MPU_START}\r\n");
                ESP_LOGI(TAG, "姿态检测已启动");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("姿态检测已启动");
                return true;
            });

        mcp_server.AddTool("self.stm32.mpu6050.stop",
            "Stop MPU6050 attitude detection",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                mpu6050_active_ = false;
                SendCommand("{CMD:MPU_STOP}\r\n");
                ESP_LOGI(TAG, "姿态检测已停止");
                auto display = Board::GetInstance().GetDisplay();
                if (display) display->ShowNotification("姿态检测已停止");
                return true;
            });
  // ============ 烟雾检测 ============          // ← 从这里开始添加
    mcp_server.AddTool("self.stm32.mq2.get_state",    // ← 从这里开始新添加
                "Get the current MQ2 smoke detection data (PPM and alarm level)",
                PropertyList(),
                [this](const PropertyList& properties) -> ReturnValue {
                    char json[256];
                    snprintf(json, sizeof(json),
                        "{\"active\": %s, \"ppm\": %.1f, \"alarm_level\": %d}",
                        mq2_active_ ? "true" : "false",
                        latest_mq2_ppm_,
                        latest_mq2_alarm_);
                    return std::string(json);
                });     
          mcp_server.AddTool("self.stm32.mq2.start",
              "Start MQ2 smoke detection",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  mq2_active_ = true;
                  SendCommand("{CMD:MQ2_START}\r\n");
                  ESP_LOGI(TAG, "烟雾检测已启动");
                  auto display = Board::GetInstance().GetDisplay();
                  if (display) display->ShowNotification("烟雾检测已启动");
                  return true;
              });

          mcp_server.AddTool("self.stm32.mq2.stop",
              "Stop MQ2 smoke detection",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  mq2_active_ = false;
                  SendCommand("{CMD:MQ2_STOP}\r\n");
                  ESP_LOGI(TAG, "烟雾检测已停止");
                  auto display = Board::GetInstance().GetDisplay();
                  if (display) display->ShowNotification("烟雾检测已停止");
                  return true;
              });
               // ============ GPS定位 ============
          mcp_server.AddTool("self.stm32.gps.get_state",
              "Get the current GPS location data (latitude, longitude, satellites)",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  char json[256];
                  snprintf(json, sizeof(json),
                      "{\"active\": %s, \"latitude\": %.6f, \"longitude\": %.6f, \"satellites\": %d}",
                      gps_active_ ? "true" : "false",
                      latest_latitude_,
                      latest_longitude_,
                      latest_satellites_);
                  return std::string(json);
              });

          mcp_server.AddTool("self.stm32.gps.start",
             "Start GPS navigation and coordinate monitoring",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  gps_active_ = true;
                  SendCommand("{CMD:GPS_START}\r\n");
                  ESP_LOGI(TAG, "GPS定位已启动");
                  auto display = Board::GetInstance().GetDisplay();
                  if (display) display->ShowNotification("GPS定位已启动");
                  return true;
              });

          mcp_server.AddTool("self.stm32.gps.stop",
              "Stop Start GPS navigation and coordinate monitoring",
              PropertyList(),
              [this](const PropertyList& properties) -> ReturnValue {
                  gps_active_ = false;
                  SendCommand("{CMD:GPS_STOP}\r\n");
                  ESP_LOGI(TAG, "GPS定位已停止");
                  auto display = Board::GetInstance().GetDisplay();
                  if (display) display->ShowNotification("GPS定位已停止");
                  return true;
              });            
        ESP_LOGI(TAG, "STM32Controller initialized successfully");
    }
};

#endif // __STM32_CONTROLLER_H__
