#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <button.hpp>
#include <ili9341.hpp>
#include <mcp3426.hpp>

#define TAG "FIGARO TGS2602"
#include "log.h"
using namespace LCD;

extern "C" {
void app_main();
}

MCP342X *adc;
esp_err_t err;

#define I2C_EXAMPLE_MASTER_SCL_IO ((gpio_num_t)22) /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO ((gpio_num_t)21) /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0		 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0		 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000		 /*!< I2C master clock frequency */

esp_err_t initI2C(i2c_port_t i2c_port) {
	i2c_config_t conf;
	conf.mode			  = I2C_MODE_MASTER;
	conf.sda_io_num	  = I2C_EXAMPLE_MASTER_SDA_IO;
	conf.sda_pullup_en	  = GPIO_PULLUP_ENABLE;
	conf.scl_io_num	  = I2C_EXAMPLE_MASTER_SCL_IO;
	conf.scl_pullup_en	  = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
	ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode,
								I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
								I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0));

	return i2c_set_timeout(i2c_port, 0xFFFFF);
}

// 7パターンを2秒間隔で読み取る
// ⇒ 1パターン当たり抵抗設定＋パラメータ読みで14プロセス/2sec行う
// ⇒ 142.857msec毎に処理
// 処理が142.857msecを超えたらどうする？
TickType_t period_tick = (int)(142.857 * portTICK_PERIOD_MS) / 1000;

gpio_num_t adj[3]	  = {GPIO_NUM_17, GPIO_NUM_5, GPIO_NUM_26};
uint8_t adj_values[7] = {
    0b000,  // 31k
    0b001,  // 26k
    0b010,  // 21k
    0b011,  // 16k
    0b101,  // 11k
    0b110,  //  6k
    0b111,  //  1k
};

void app_main() {
	_i("Start");

	ILI9341 *lcd = new ILI9341();

	err = initI2C(I2C_NUM_0);
	adc = new MCP342X(I2C_NUM_0, 0x68);

	for (int i = 0; i < 3; i++) {
		gpio_set_direction(adj[i], GPIO_MODE_OUTPUT);
		gpio_set_level(adj[i], 0);
	}

	// センサー値取得
	xTaskCreatePinnedToCore([](void *_lcd) {
		_i("Check values on Core %d", xPortGetCoreID());

		ILI9341 *lcd = (ILI9341 *)_lcd;
		char buf[64] = {0};

		int mode = 0;

		int32_t values[7] = {0};

		TickType_t current = xTaskGetTickCount();

		uint16_t *frameBuffer = lcd->getBuffer();

		int refresh = 0;

		lcd->clear(BLACK);
		lcd->drawString(10, 100, WHITE, "Initializing...");
		lcd->update();
		lcd->clear(BLACK);

		while (true) {
			// Core1はWatch Dog Timerを介入させるためにDelay挿入
			vTaskDelay(1);
			TickType_t now = xTaskGetTickCount();
			if (now - current < period_tick) continue;
			current = now;

			_i("Mode [%d], Tick %d,", mode, current);

			if (mode >= 0b1110) {
				// ディスプレイ更新モード
				lcd->update();

				lcd->clear(BLACK);

				for (int x = 315; x < 320; x++)
					for (int y = 0; y < 5; y++)
						frameBuffer[y * 320 + x] = refresh ? GREEN : RED;

				// for(int i=0; i<7; i++) values[i] = 0;
				mode	   = 0;
				refresh = 1 - refresh;
				continue;
			}

			if (!(mode & 0b1)) {
				// インピーダンス変更モード
				for (int i = 0; i < 3; i++) {
					gpio_set_level(adj[i], (adj_values[mode >> 1] >> i) & 0b1);
					_i("GPIO [%d]: to %d", adj[i], (adj_values[mode >> 1] >> i) & 0b1);
				}
			} else {
				// 読み取りモード
				int t	  = mode >> 1;
				esp_err_t e = adc->getValues(&values[t]);
				if (e)
					sprintf(buf, "%d: Error", t);
				else
					sprintf(buf, "%d: %5d", t, values[t]);
				lcd->drawString(10, 5 + 14 * t, GREEN, buf);
			}
			mode += 1;
		}
	}, "TGS", 4096, lcd, 1, NULL, 1);
}
