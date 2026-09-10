################################################################################
# Copyright (c) 2024,D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import os
import os.path
import copy


HOBOT_PI = 'HOBOT_PI'
SYSFS_GPIO = "/sys/class/gpio"
SYSFS_PLATFORM_PATH = '/sys/devices/platform/'
SYSFS_BOARDID_PATH = '/sys/class/socinfo/board_id'
SYSFS_SOCNAME_PATH = '/sys/class/socinfo/soc_name'
HOBOT_PI_PATTERN = 'hobot,x3'

# [0]- GPIO chip sysfs directory
# [1]- Linux exported GPIO name,
# [2]- Pin number (BOARD mode)
# [3]- Pin number (BCM mode)
# [4]- Pin name (CVM mode)
# [5]- Pin name (SOC mode)
# [6]- PWM chip sysfs directory
# [7]- PWM ID

SDBV3_PIN = [
    ["soc/a6003000.gpio", 11, 3, 2, 'I2C1_SDA', 'I2C1_SDA', None, None],
    ["soc/a6003000.gpio", 10, 5, 3, 'I2C1_SCL', 'I2C1_SCL', None, None],
    ["soc/a6003000.gpio", 38, 7, 4, 'GPIO38', 'GPIO38', None, None],
    ["soc/a6003000.gpio", 6, 11, 17, 'GPIO6', 'JTG_TDI', None, None],
    ["soc/a6003000.gpio", 5, 13, 27, 'GPIO5', 'JTG_TMS', None, None],
    ["soc/a6003000.gpio", 30, 15, 22, 'GPIO30', 'BIFSPI_MISO', None, None],
    ["soc/a6003000.gpio", 18, 19, 10, 'SPI0_MOSI', 'SPI0_MOSI', "soc/a500e000.pwm", 2],
    ["soc/a6003000.gpio", 19, 21, 9, 'SPI0_MISO', 'SPI0_MISO', "soc/a500f000.pwm", 0],
    ["soc/a6003000.gpio", 17, 23, 11, 'SPI0_SCLK', 'SPI0_SCLK', None, None],
    ["soc/a6003000.gpio", 14, 29, 5, 'GPIO14', 'SPI2_SCLK', None, None],
    ["soc/a6003000.gpio", 31, 31, 6, 'GPIO31', 'BIFSPI_RSTN', None, None],
    ["soc/a6003000.gpio", 13, 33, 13, 'PWM8', 'SPI2_MISO', "soc/a500f000.pwm", 2],
    ["soc/a6003000.gpio", 103, 35, 19, 'I2S0_LRCK', 'I2S0_LRCK', None, None],
    ["soc/a6003000.gpio", 29, 37, 26, 'GPIO29', 'BIFSPI_MOSI', None, None],

    ["soc/a6003000.gpio", 95, 8, 14, 'UART0_TXD', 'UART0_TXD', None, None],
    ["soc/a6003000.gpio", 96, 10, 15,'UART0_RXD', 'UART0_RXD', None, None],
    ["soc/a6003000.gpio", 102, 12, 18, 'I2S0_BCLK', 'I2S0_BCLK', None, None],
    ["soc/a6003000.gpio", 27, 16, 23, 'GPIO27', 'BIFSPI_CSN', None, None],
    ["soc/a6003000.gpio", 7, 18, 24, 'GPIO7', 'JTG_TDO', None, None],
    ["soc/a6003000.gpio", 15, 22, 25, 'GPIO15', 'SPI2_CSN', None, None],
    ["soc/a6003000.gpio", 16, 24, 8, 'SPI0_CSN', 'I2C4_SDA', None, None],
    ["soc/a6003000.gpio", 120, 26, 7, 'SPI0_CSN1', 'QSPI_CSN1', None, None],
    ["soc/a6003000.gpio", 12, 32, 12, 'PWM7', 'SPI2_MOSI', "soc/a500f000.pwm", 1],
    ["soc/a6003000.gpio", 28, 36, 16, 'GPIO28', 'BIFSPI_SCLK', None, None],
    ["soc/a6003000.gpio", 104, 38, 20, 'I2S0_SDIO', 'I2S0_SDIO', None, None],
    ["soc/a6003000.gpio", 108, 40, 21, 'I2S1_SDIO', 'I2S1_SDIO', None, None],
]

SDB_PIN = [
    ["soc/a6003000.gpio", 11, 3, 2, 'I2C1_SDA', 'I2C1_SDA', None, None],
    ["soc/a6003000.gpio", 10, 5, 3, 'I2C1_SCL', 'I2C1_SCL', None, None],
    ["soc/a6003000.gpio", 101, 7, 4, 'I2S0_MCLK', 'I2S0_MCLK', None, None],
    ["soc/a6003000.gpio", 6, 11, 17, 'GPIO6', 'JTG_TDI', None, None],
    ["soc/a6003000.gpio", 5, 13, 27, 'GPIO5', 'JTG_TMS', None, None],
    ["soc/a6003000.gpio", 30, 15, 22, 'GPIO30', 'BIFSPI_MISO', None, None],
    ["soc/a6003000.gpio", 18, 19, 10, 'SPI0_MOSI', 'SPI0_MOSI', "soc/a500e000.pwm", 2],
    ["soc/a6003000.gpio", 19, 21, 9, 'SPI0_MISO', 'SPI0_MISO', "soc/a500f000.pwm", 0],
    ["soc/a6003000.gpio", 17, 23, 11, 'SPI0_SCLK', 'SPI0_SCLK', None, None],
    ["soc/a6003000.gpio", 106, 27, 0, 'I2S1_BLCK', 'I2S1_BLCK', None, None],
    ["soc/a6003000.gpio", 14, 29, 5, 'GPIO14', 'SPI2_SCLK', None, None],
    ["soc/a6003000.gpio", 31, 31, 6, 'GPIO31', 'BIFSPI_RSTN', None, None],
    ["soc/a6003000.gpio", 4, 33, 13, 'PWM0', 'PWM0', "soc/a500d000.pwm", 0],
    ["soc/a6003000.gpio", 103, 35, 19, 'I2S0_LRCK', 'I2S0_LRCK', None, None],
    ["soc/a6003000.gpio", 105, 37, 26, 'GPIO105', 'I2S1_MCLK', None, None],

    ["soc/a6003000.gpio", 111, 8, 14, 'UART3_TXD', 'UART3_TXD', None, None],
    ["soc/a6003000.gpio", 112, 10, 15,'UART3_RXD', 'UART3_RXD', None, None],
    ["soc/a6003000.gpio", 102, 12, 18, 'I2S0_BCLK', 'I2S0_BCLK', None, None],
    ["soc/a6003000.gpio", 27, 16, 23, 'GPIO27', 'BIFSPI_CSN', None, None],
    ["soc/a6003000.gpio", 7, 18, 24, 'GPIO7', 'JTG_TDO', None, None],
    ["soc/a6003000.gpio", 15, 22, 25, 'GPIO15', 'SPI2_CSN', None, None],
    ["soc/a6003000.gpio", 16, 24, 8, 'SPI0_CSN', 'I2C4_SDA', None, None],
    ["soc/a6003000.gpio", 120, 26, 7, 'SPI0_CSN1', 'QSPI_CSN1', None, None],
    ["soc/a6003000.gpio", 107, 28, 1, 'I2S1_LRCK', 'I2S1_LRCK', None, None],
    ["soc/a6003000.gpio", 25, 32, 12, 'PWM4', 'PWM4', "soc/a500e000.pwm", 1],
    ["soc/a6003000.gpio", 3, 36, 16, 'GPIO3', 'JTG_TCK', None, None],
    ["soc/a6003000.gpio", 104, 38, 20, 'I2S0_SDIO', 'I2S0_SDIO', None, None],
    ["soc/a6003000.gpio", 108, 40, 21, 'I2S1_SDIO', 'I2S1_SDIO', None, None],
]

X3_PI_PIN = [
    ["soc/a6003000.gpio", 9, 3, 2, 'I2C0_SDA', 'I2C0_SDA', None, None],
    ["soc/a6003000.gpio", 8, 5, 3,'I2C0_SCL', 'I2C0_SCL', None, None],
    ["soc/a6003000.gpio", 101, 7, 4, 'I2S0_MCLK', 'I2S0_MCLK', None, None],
    ["soc/a6003000.gpio", 6, 11, 17, 'GPIO6', 'JTG_TDI', None, None],
    ["soc/a6003000.gpio", 5, 13, 27,'GPIO5', 'JTG_TMS', None, None],
    ["soc/a6003000.gpio", 30, 15, 22, 'GPIO30', 'BIFSPI_MISO', None, None],
    ["soc/a6003000.gpio", 12, 19, 10, 'SPI2_MOSI', 'SPI2_MOSI', "soc/a500f000.pwm", 1],
    ["soc/a6003000.gpio", 13, 21, 9, 'SPI2_MISO', 'SPI2_MISO', "soc/a500f000.pwm", 2],
    ["soc/a6003000.gpio", 14, 23, 11, 'SPI2_SCLK', 'SPI2_SCLK', None, None],
    ["soc/a6003000.gpio", 106, 27, 0, 'I2S1_BCLK', 'I2S1_BCLK', None, None],
    ["soc/a6003000.gpio", 119, 29, 5, 'GPIO119', 'GPIO119', None, None],
    ["soc/a6003000.gpio", 118, 31, 6, 'GPIO118', 'GPIO118', None, None],
    ["soc/a6003000.gpio", 4, 33, 13, 'PWM0', 'JTG_TRSTN', "soc/a500d000.pwm", 0],
    ["soc/a6003000.gpio", 103, 35, 19, 'I2S0_LRCK', 'I2S0_LRCK', None, None],
    ["soc/a6003000.gpio", 105, 37, 26, 'GPIO105', 'I2S1_MCLK', None, None],

    ["soc/a6003000.gpio", 111, 8, 14, 'UART_TXD', 'SENSOR2_MCLK', None, None],
    ["soc/a6003000.gpio", 112, 10, 15, 'UART_RXD', 'SENSOR3_MCLK', None, None],
    ["soc/a6003000.gpio", 102, 12, 18, 'I2S0_BCLK', 'I2S0_BCLK', None, None],
    ["soc/a6003000.gpio", 27, 16, 23, 'GPIO27', 'BIFSPI_CSN', None, None],
    ["soc/a6003000.gpio", 7, 18, 24, 'GPIO7', 'JTG_TDO', None, None],
    ["soc/a6003000.gpio", 29, 22, 25, 'GPIO29', 'BIFSPI_MOSI', None, None],
    ["soc/a6003000.gpio", 15, 24, 8, 'SPI2_CSN', 'SPI2_CSN', None, None],
    ["soc/a6003000.gpio", 28, 26, 7, 'GPIO28', 'BIFSPI_SCLK', None, None],
    ["soc/a6003000.gpio", 107, 28, 1, 'I2S1_LRCK', 'I2S1_LRCK', None, None],
    ["soc/a6003000.gpio", 25, 32, 12, 'PWM4', 'PWM4', "soc/a500e000.pwm", 1],
    ["soc/a6003000.gpio", 3, 36, 16, 'GPIO3', 'JTG_TCK', None, None],
    ["soc/a6003000.gpio", 104, 38, 20, 'I2S0_SDIO', 'I2S0_SDIO', None, None],
    ["soc/a6003000.gpio", 108, 40, 21, 'I2S1_SDIO', 'I2S1_SDIO', None, None],
]

X3_CM_PIN = [
    ["soc/a6003000.gpio", 9, 3, 2, 'I2C0_SDA', 'I2C0_SDA', None, None],
    ["soc/a6003000.gpio", 8, 5, 3,'I2C0_SCL', 'I2C0_SCL', None, None],
    ["soc/a6003000.gpio", 101, 7, 4, 'I2S0_MCLK', 'I2S0_MCLK', None, None],
    ["soc/a6003000.gpio", 12, 11, 17, 'GPIO17', 'SPI2_MOSI', "soc/a500f000.pwm", 1],
    ["soc/a6003000.gpio", 13, 13, 27,'GPIO27', 'SPI2_MISO', "soc/a500f000.pwm", 2],
    ["soc/a6003000.gpio", 30, 15, 22, 'GPIO22', 'BIFSPI_MISO', None, None],
    ["soc/a6003000.gpio", 6, 19, 10, 'SPI1_MOSI', 'SPI1_MOSI', None, None],
    ["soc/a6003000.gpio", 7, 21, 9, 'SPI1_MISO', 'SPI1_MISO', None, None],
    ["soc/a6003000.gpio", 3, 23, 11, 'SPI1_SCLK', 'SPI1_SCLK', None, None],
    ["soc/a6003000.gpio", 15, 27, 0, 'I2C3_SDA', 'I2C3_SDA', None, None],
    ["soc/a6003000.gpio", 119, 29, 5, 'GPIO5', 'LPWM3', None, None],
    ["soc/a6003000.gpio", 118, 31, 6, 'GPIO6', 'LPWM4', None, None],
    ["soc/a6003000.gpio", 4, 33, 13, 'PWM0', 'PWM0', "soc/a500d000.pwm", 0],
    ["soc/a6003000.gpio", 103, 35, 19, 'I2S0_LRCK', 'I2S0_LRCK', None, None],
    ["soc/a6003000.gpio", 117, 37, 26, 'GPIO25', 'LPWM5', None, None],

    ["soc/a6003000.gpio", 111, 8, 14, 'UART_TXD', 'UART3_TXD', None, None],
    ["soc/a6003000.gpio", 112, 10, 15, 'UART_RXD', 'UART3_RXD', None, None],
    ["soc/a6003000.gpio", 102, 12, 18, 'I2S0_BCLK', 'I2S0_BCLK', None, None],
    ["soc/a6003000.gpio", 27, 16, 23, 'GPIO23', 'BIFSPI_CSN', None, None],
    ["soc/a6003000.gpio", 22, 18, 24, 'GPIO24', 'PWM1', "soc/a500d000.pwm", 1],
    ["soc/a6003000.gpio", 29, 22, 25, 'GPIO25', 'BIFSPI_MOSI', None, None],
    ["soc/a6003000.gpio", 5, 24, 8, 'SPI1_CSN', 'SPI1_CSN', None, None],
    ["soc/a6003000.gpio", 28, 26, 7, 'GPIO7', 'BIFSPI_SCLK', None, None],
    ["soc/a6003000.gpio", 14, 28, 1, 'I2C3_SCL', 'I2C3_SCL', None, None],
    ["soc/a6003000.gpio", 25, 32, 12, 'PWM4', 'PWM4', "soc/a500e000.pwm", 1],
    ["soc/a6003000.gpio", 20, 36, 16, 'GPIO16', 'BIFSD_CLK', None, None],
    ["soc/a6003000.gpio", 108, 38, 20, 'I2S1_SDIO', 'I2S1_SDIO', None, None],    
    ["soc/a6003000.gpio", 104, 40, 21, 'I2S0_SDIO', 'I2S0_SDIO', None, None],
]

X3_PI_V2_1_PIN = [
    ["soc/a6003000.gpio", 9, 3, 2, 'I2C0_SDA', 'I2C0_SDA', None, None],
    ["soc/a6003000.gpio", 8, 5, 3,'I2C0_SCL', 'I2C0_SCL', None, None],
    ["soc/a6003000.gpio", 101, 7, 4, 'I2S0_MCLK', 'I2S0_MCLK', None, None],
    ["soc/a6003000.gpio", 12, 11, 17, 'GPIO17', 'SPI2_MOSI', "soc/a500f000.pwm", 1],
    ["soc/a6003000.gpio", 13, 13, 27,'GPIO27', 'SPI2_MISO', "soc/a500f000.pwm", 2],
    ["soc/a6003000.gpio", 30, 15, 22, 'GPIO22', 'BIFSPI_MISO', None, None],
    ["soc/a6003000.gpio", 6, 19, 10, 'SPI1_MOSI', 'SPI1_MOSI', None, None],
    ["soc/a6003000.gpio", 7, 21, 9, 'SPI1_MISO', 'SPI1_MISO', None, None],
    ["soc/a6003000.gpio", 3, 23, 11, 'SPI1_SCLK', 'SPI1_SCLK', None, None],
    ["soc/a6003000.gpio", 15, 27, 0, 'I2C3_SDA', 'I2C3_SDA', None, None],
    ["soc/a6003000.gpio", 119, 29, 5, 'GPIO5', 'LPWM3', None, None],
    ["soc/a6003000.gpio", 118, 31, 6, 'GPIO6', 'LPWM4', None, None],
    ["soc/a6003000.gpio", 4, 33, 13, 'PWM0', 'PWM0', "soc/a500d000.pwm", 0],
    ["soc/a6003000.gpio", 103, 35, 19, 'I2S0_LRCK', 'I2S0_LRCK', None, None],
    ["soc/a6003000.gpio", 117, 37, 26, 'GPIO25', 'LPWM5', None, None],

    ["soc/a6003000.gpio", 111, 8, 14, 'UART_TXD', 'UART3_TXD', None, None],
    ["soc/a6003000.gpio", 112, 10, 15, 'UART_RXD', 'UART3_RXD', None, None],
    ["soc/a6003000.gpio", 102, 12, 18, 'I2S0_BCLK', 'I2S0_BCLK', None, None],
    ["soc/a6003000.gpio", 27, 16, 23, 'GPIO23', 'BIFSPI_CSN', None, None],
    ["soc/a6003000.gpio", 22, 18, 24, 'GPIO24', 'PWM1', "soc/a500d000.pwm", 1],
    ["soc/a6003000.gpio", 29, 22, 25, 'GPIO25', 'BIFSPI_MOSI', None, None],
    ["soc/a6003000.gpio", 5, 24, 8, 'SPI1_CSN', 'SPI1_CSN', None, None],
    ["soc/a6003000.gpio", 28, 26, 7, 'GPIO7', 'BIFSPI_SCLK', None, None],
    ["soc/a6003000.gpio", 14, 28, 1, 'I2C3_SCL', 'I2C3_SCL', None, None],
    ["soc/a6003000.gpio", 25, 32, 12, 'PWM4', 'PWM4', "soc/a500e000.pwm", 1],
    ["soc/a6003000.gpio", 20, 36, 16, 'GPIO16', 'BIFSD_CLK', None, None],
    ["soc/a6003000.gpio", 104, 38, 20, 'I2S0_SDIO', 'I2S0_SDIO', None, None],
    ["soc/a6003000.gpio", 108, 40, 21, 'I2S1_SDIO', 'I2S1_SDIO', None, None],
]

RDK_X5_PIN = [
    ["soc/34000000.a55_apb0/34120000.gpio", 390, 3, 2, 'SDA', 'I2C5_SDA', None, None],                              #I2C5_SDA/UART3_TXD
    ["soc/34000000.a55_apb0/34120000.gpio", 389, 5, 3,'SCL', 'I2C5_SCL', None, None],                               #I2C5_SCL/UART3_RXD
    ["soc/32080000.dsp_apb/32150000.gpio", 420, 7, 4, 'GPCLK0', 'I2S1_MCLK', None, None],                           #I2S1_MCLK
    ["soc/34000000.a55_apb0/34120000.gpio", 380, 11, 17, 'GPIO17', 'UART7_TXD', None, None],                        #UART7_TXD
    ["soc/34000000.a55_apb0/34120000.gpio", 379, 13, 27,'GPIO27', 'UART7_RXD', None, None],                         #UART7_RXD
    ["soc/34000000.a55_apb0/34120000.gpio", 388, 15, 22, 'GPIO22', 'UART2_TXD', None, None],                        #UART2_TXD
    ["soc/34000000.a55_apb0/34120000.gpio", 398, 19, 10, 'SPI_MOSI', 'SPI1_MOSI', None, None],                      #SPI1_MOSI/JTG_TDO
    ["soc/34000000.a55_apb0/34120000.gpio", 397, 21, 9, 'SPI_MISO', 'SPI1_MISO', None, None],                       #SPI1_MISO/JTG_TDI
    ["soc/34000000.a55_apb0/34120000.gpio", 395, 23, 11, 'SPI_SCLK', 'SPI1_SCLK', None, None],                      #SPI1_SCLK/JTG_TCK
    ["soc/34000000.a55_apb0/34130000.gpio", 355, 27, 0, 'ID_SD', 'I2C0_SDA', "soc/34000000.a55_apb0/34160000.pwm", 1],                           #I2C0_SDA/PWM5
    ["soc/34000000.a55_apb0/34120000.gpio", 399, 29, 5, 'GPIO5', 'SPI2_SCLK', "soc/34000000.a55_apb0/34140000.pwm", 0],                          #SPI2_SCLK/PWM0
    ["soc/34000000.a55_apb0/34120000.gpio", 400, 31, 6, 'GPIO6', 'I2C1_SDA', "soc/34000000.a55_apb0/34140000.pwm", 1],                           #I2C1_SDA/PWM1
    ["soc/34000000.a55_apb0/34130000.gpio", 357, 33, 13, 'PWM', 'PWM7', "soc/34000000.a55_apb0/34170000.pwm",1],    #PWM7/I2C1_SDA
    ["soc/32080000.dsp_apb/32150000.gpio", 422, 35, 19, 'PCM_FS', 'I2S1_LRCK', None, None],                         #I2S1_LRCK
    ["soc/34000000.a55_apb0/34120000.gpio", 401, 37, 26, 'GPIO26', 'SPI2_MISO', "soc/34000000.a55_apb0/34150000.pwm", 0],                        #SPI2_MISO

    ["soc/34000000.a55_apb0/34120000.gpio", 383, 8, 14, 'TXD', 'UART1_TXD', None, None],                            #UART1_TXD
    ["soc/34000000.a55_apb0/34120000.gpio", 384, 10, 15, 'RXD', 'UART1_RXD', None, None],                           #UART1_RXD
    ["soc/32080000.dsp_apb/32150000.gpio", 421, 12, 18, 'PCM_CLK', 'I2S1_BCLK', None, None],                        #I2S1_BCLK
    ["soc/34000000.a55_apb0/34120000.gpio", 382, 16, 23, 'GPIO23', 'UART6_TXD', None, None],                        #UART6_TXD/UART7_RTS
    ["soc/34000000.a55_apb0/34120000.gpio", 402, 18, 24, 'GPIO24', 'SPI2_MOSI', "soc/34000000.a55_apb0/34150000.pwm", 1],                        #SPI2_MOSI/PWM3
    ["soc/34000000.a55_apb0/34120000.gpio", 387, 22, 25, 'GPIO25', 'UART2_RXD', None, None],                        #UART2_RXD
    ["soc/34000000.a55_apb0/34120000.gpio", 394, 24, 8, 'SPI_CSN0', 'SPI1_CSN1', None, None],                       #SPI1_CSN1/JTG_TMS
    ["soc/34000000.a55_apb0/34120000.gpio", 396, 26, 7, 'SPI_CSN1', 'SPI1_CSN0', None, None],                       #SPI1_CSN0/JTG_TRSTN
    ["soc/34000000.a55_apb0/34130000.gpio", 354, 28, 1, 'ID_SC', 'I2C0_SCL', "soc/34000000.a55_apb0/34160000.pwm", 0],                           #I2C0_SCL/PWM4
    ["soc/34000000.a55_apb0/34130000.gpio", 356, 32, 12, 'PWM', 'PWM6', "soc/34000000.a55_apb0/34170000.pwm",0],    #PWM6/I2C1_SCL/TIME_SYNC1
    ["soc/34000000.a55_apb0/34120000.gpio", 381, 36, 16, 'GPIO16', 'BIFSD_CLK', None, None],                        #UART6_RXD/UART7_CTS
    ["soc/32080000.dsp_apb/32150000.gpio", 423, 38, 20, 'PCM_DIN', 'I2S1_SDIN', None, None],                        #I2S1_SDIN
    ["soc/32080000.dsp_apb/32150000.gpio", 424, 40, 21, 'PCM_DOUT', 'I2S1_SDOUT', None, None],                      #I2S1_SDOUT
]

EVB_X5_PIN = [
    ["soc/34000000.a55_apb0/34120000.gpio", 390, 3, 2, 'SDA5', 'I2C5_SDA', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 389, 5, 3,'SCL5', 'I2C5_SCL', None, None],
    ["soc/32080000.dsp_apb/32150000.gpio", 420, 7, 4, 'I2S1_MCLK', 'DSP_MCLK1', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 382, 11, 17, 'GPIO17', 'UART7_RTS', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 381, 13, 27,'GPIO27', 'UART7_CTS', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 385, 15, 22, 'GPIO22', 'UART1_CTS', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 402, 19, 10, 'SPI2_MOSI', 'SPI2_MOSI', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 401, 21, 9, 'SPI2_MISO', 'SPI2_MISO', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 399, 23, 11, 'SPI2_SCLK', 'SPI2_SCLK', None, None],
    ["soc/34000000.a55_apb0/34130000.gpio", 357, 27, 0, 'SDA1', 'I2C1_SDA', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 379, 29, 5, 'GPIO5', 'UART7_RX', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 380, 31, 6, 'GPIO6', 'UART7_TX', None, None],
    ["soc/34000000.a55_apb0/34130000.gpio", 354, 33, 13, 'PWM4', 'PWM4', "soc/34000000.a55_apb0/34160000.pwm",0], 
    ["soc/32080000.dsp_apb/32150000.gpio", 422, 35, 19, 'I2S1_WS', 'I2S1_WS', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 386, 37, 26, 'GPIO26', 'UART1_RTS', None, None],

    ["soc/34000000.a55_apb0/34120000.gpio", 387, 8, 14, 'TXD', 'UART2_TXD', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 388, 10, 15, 'RXD', 'UART2_RXD', None, None],
    ["soc/32080000.dsp_apb/32150000.gpio", 421, 12, 18, 'I2S1_BCLK', 'I2S1_BCLK', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 383, 16, 23, 'GPIO23', 'UART1_RXD', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 384, 18, 24, 'GPIO24', 'UART1_TXD', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 391, 22, 25, 'GPIO25', 'UART4_RXD', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 400, 24, 8, 'SPI2_CSN', 'SPI2_CSN', None, None],
    ["soc/34000000.a55_apb0/34120000.gpio", 392, 26, 7, 'GPIO7', 'UART4_TXD', None, None],
    ["soc/34000000.a55_apb0/34130000.gpio", 356, 28, 1, 'SCL1', 'I2C1_SCL', None, None],
    ["soc/34000000.a55_apb0/34130000.gpio", 355, 32, 12, 'PWM', 'PWM5', "soc/34000000.a55_apb0/34160000.pwm",1],
    ["soc/32080000.dsp_apb/32150000.gpio", 423, 38, 20, 'I2S1_DI', 'I2S1_SDIN', None, None],
    ["soc/32080000.dsp_apb/32150000.gpio", 424, 40, 21, 'I2S1_DO', 'I2S1_SDOUT', None, None],
]

ALL_BOARD_DATA_X3 = [
    {'board_name' : 'X3SDBV3', 'pin_info' : SDBV3_PIN, 'board_id' : 0x304},
    {'board_name' : 'X3SDB', 'pin_info' : SDB_PIN, 'board_id' : 0x404},
    {'board_name' : 'X3PI', 'pin_info' : X3_PI_PIN, 'board_id' : 0x504},
    {'board_name' : 'X3PI', 'pin_info' : X3_PI_PIN, 'board_id' : 0x604},
    {'board_name' : 'X3CM', 'pin_info' : X3_CM_PIN, 'board_id' : 0xb04},
    {'board_name' : 'X3PI_V2_1', 'pin_info' : X3_PI_V2_1_PIN, 'board_id' : 0x804},
]

ALL_BOARD_DATA_X5 = [
    {'board_name' : 'RDK_X5', 'pin_info' : RDK_X5_PIN, 'board_id' : 0x301},
    {'board_name' : 'RDK_X5', 'pin_info' : RDK_X5_PIN, 'board_id' : 0x302},
    {'board_name' : 'RDK_X5', 'pin_info' : RDK_X5_PIN, 'board_id' : 0x501},
    {'board_name' : 'RDK_X5', 'pin_info' : RDK_X5_PIN, 'board_id' : 0x502},
    {'board_name' : 'RDK_X5', 'pin_info' : RDK_X5_PIN, 'board_id' : 0x503},
    {'board_name' : 'RDK_X5', 'pin_info' : RDK_X5_PIN, 'board_id' : 0x504},
    {'board_name' : 'RDK_X5', 'pin_info' : RDK_X5_PIN, 'board_id' : 0x505},
    {'board_name' : 'RDK_X5', 'pin_info' : RDK_X5_PIN, 'board_id' : 0x506},
    {'board_name' : 'EVB_X5', 'pin_info' : EVB_X5_PIN, 'board_id' : 0x201}, #HOBOT_X5_EVB_LP4_1_A_ID
    {'board_name' : 'EVB_X5', 'pin_info' : EVB_X5_PIN, 'board_id' : 0x202}, #HOBOT_X5_EVB_LP4_1_B_ID
    {'board_name' : 'EVB_X5', 'pin_info' : EVB_X5_PIN, 'board_id' : 0x203}, #HOBOT_X5_EVB_LP4_V1P2_ID
    {'board_name' : 'EVB_X5', 'pin_info' : EVB_X5_PIN, 'board_id' : 0x204}, #HOBOT_X5_EVB_LP4_V1P3_ID
]


class AllInfo(object):
    def __init__(self, gpio_chip_dir, gpio_id, pwm_chip_dir, pwm_id):
        self.gpio_chip_dir = gpio_chip_dir
        self.gpio_id = gpio_id
        self.pwm_chip_dir = pwm_chip_dir
        self.pwm_id = pwm_id


def get_all_pin_data():
    if (not os.access(SYSFS_GPIO + '/export', os.W_OK)):
        raise RuntimeError("Insufficient permissions, need root permissions:" + SYSFS_GPIO + '/export')
    if (not os.access(SYSFS_GPIO + '/unexport', os.W_OK)):
        raise RuntimeError("Insufficient permissions, need root permissions")
    if (not os.access(SYSFS_BOARDID_PATH, os.R_OK)):
        raise RuntimeError("Insufficient permissions, need root permissions")
    if (not os.access(SYSFS_BOARDID_PATH, os.R_OK)):
        raise RuntimeError("Insufficient permissions, need root permissions")

    res = False
    compatible_path = '/proc/device-tree/compatible'
    with open(compatible_path, 'r') as f:
        compatibles = f.read().split('\x00')
    with open(SYSFS_BOARDID_PATH, 'r') as f:
        sboard_id = "0x" + f.read()
        iboard_id = int(sboard_id,16)
        board_id = iboard_id & 0xfff
    with open(SYSFS_SOCNAME_PATH, 'r') as f:
        soc_name = f.read().strip()
    if 'x5' in soc_name.lower():
        board_list = ALL_BOARD_DATA_X5
    else:
        board_list = ALL_BOARD_DATA_X3
    for board_data in board_list:
        if board_data['board_id'] == board_id:
            pin_data = copy.deepcopy(board_data['pin_info'])
            model = board_data['board_name']
            res = True
            break
    if not res:
        raise Exception("Board type is not support")
    # Check GPIO chip dir
    for n, x in enumerate(pin_data):
        if x[0] is None:
            continue
        gpio_chip_name = x[0]
        gpio_chip_dir = SYSFS_PLATFORM_PATH + gpio_chip_name
        gpio_chip_dir = gpio_chip_dir + '/gpio'
        if not os.path.exists(gpio_chip_dir):
            continue
        for f in os.listdir(gpio_chip_dir):
            if not f.startswith('gpiochip'):
                continue
            gpio_chip_dir = gpio_chip_dir + '/' + f
            pin_data[n][0] = gpio_chip_dir
            break
    # Check PWM chip dir
    for n, x in enumerate(pin_data):
        if x[6] is None:
            continue
        pwm_chip_name = x[6]
        pwm_chip_dir = SYSFS_PLATFORM_PATH + pwm_chip_name
        pwm_chip_dir = pwm_chip_dir + '/pwm'
        if not os.path.exists(pwm_chip_dir):
            continue
        for f in os.listdir(pwm_chip_dir):
            if not f.startswith('pwmchip'):
                continue
            pwm_chip_dir = pwm_chip_dir + '/' + f
            pin_data[n][6] = pwm_chip_dir
            break

    def sort_data(pin_name, pin_data):
        return {x[pin_name]: AllInfo(
            x[0],
            x[1],
            x[6],
            x[7]) for x in pin_data}

    all_pin_data = {
        'BOARD': sort_data(2, pin_data),
        'BCM': sort_data(3, pin_data),
        'CVM': sort_data(4, pin_data),
        'SOC': sort_data(1, pin_data),
        'TEGRA_SOC': sort_data(5, pin_data),
    }
    return model, all_pin_data
