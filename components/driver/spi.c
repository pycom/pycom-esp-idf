// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>

#include "spi.h"
#include "soc/spi_reg.h"
#include "soc/rtc_cntl_reg.h"

#include "rom/ets_sys.h"

#include "esp_intr.h"
#include "esp_attr.h"
#include "soc/dport_reg.h"

//*****************************************************************************
//
// Make sure all of the definitions in this header have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Defines slave commands.Default value based on slave ESP8266 & ESP32.
 */
#define MASTER_WRITE_DATA_TO_SLAVE_CMD                      2
#define MASTER_READ_DATA_FROM_SLAVE_CMD                     3
#define MASTER_WRITE_STATUS_TO_SLAVE_CMD                    1
#define MASTER_READ_STATUS_FROM_SLAVE_CMD                   4


/**
 * @brief Based on pAttr initialize SPI module.
 *
 */
void spi_init(spi_num_e spiNum, spi_attr_t* pAttr)
{
    if ((spiNum > SpiNum_Max)
        || (NULL == pAttr)) {
        return;
    }

    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(spiNum), SPI_TRANS_DONE << 5);
    SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_CS_SETUP);

    // SPI_CPOL & SPI_CPHA
    switch (pAttr->subMode) {
    case SpiSubMode_1:
        CLEAR_PERI_REG_MASK(SPI_PIN_REG(spiNum), SPI_CK_IDLE_EDGE);
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum),  SPI_CK_OUT_EDGE); // CHPA_FALLING_EDGE_SAMPLE
        break;
    case SpiSubMode_2:
        SET_PERI_REG_MASK(SPI_PIN_REG(spiNum), SPI_CK_IDLE_EDGE);
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum),  SPI_CK_OUT_EDGE); // CHPA_FALLING_EDGE_SAMPLE
        break;
    case SpiSubMode_3:
        SET_PERI_REG_MASK(SPI_PIN_REG(spiNum), SPI_CK_IDLE_EDGE);
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum),  SPI_CK_OUT_EDGE);
        break;
    case SpiSubMode_0:
    default:
        CLEAR_PERI_REG_MASK(SPI_PIN_REG(spiNum), SPI_CK_IDLE_EDGE);
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum),  SPI_CK_OUT_EDGE);
        // To do nothing
        break;
    }

    // SPI bit order
    if (SpiBitOrder_MSBFirst == pAttr->bitOrder) {
        CLEAR_PERI_REG_MASK(SPI_CTRL_REG(spiNum), SPI_WR_BIT_ORDER);
        CLEAR_PERI_REG_MASK(SPI_CTRL_REG(spiNum), SPI_RD_BIT_ORDER);
    } else if (SpiBitOrder_LSBFirst == pAttr->bitOrder) {
        SET_PERI_REG_MASK(SPI_CTRL_REG(spiNum), SPI_WR_BIT_ORDER);
        SET_PERI_REG_MASK(SPI_CTRL_REG(spiNum), SPI_RD_BIT_ORDER);
    } else {
        // To do nothing
    }

    // SPI bit order
    if (SpiWorkMode_Half == pAttr->halfMode) {
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_DOUTDIN);
    } else if (SpiWorkMode_Full == pAttr->halfMode) {
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_DOUTDIN);
    }
    // May be not must to do.
    WRITE_PERI_REG(SPI_USER1_REG(spiNum), 0);
    // SPI mode type
    if (SpiMode_Master == pAttr->mode) {
        // SPI mode type
        SET_PERI_REG_BITS(SPI_CTRL2_REG(spiNum), SPI_MISO_DELAY_MODE, 0, SPI_MISO_DELAY_MODE_S); ////??????
        // SPI_SET_MISO_DELAY_NUM(spiNum,0);////???????
        //SET_PERI_REG_BITS(SPI_CTRL2_REG(spiNum), SPI_MISO_DELAY_NUM,0,SPI_MISO_DELAY_NUM_S);////??????

        CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(spiNum), SPI_SLAVE_MODE);
        // SPI Send buffer
        // CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MISO_HIGHPART );// By default slave send buffer C0-C7
        // SPI Speed
        if (1 < (pAttr->speed)) {
            uint8_t i, k;
            i = (pAttr->speed / 40) ? (pAttr->speed / 40) : 1;

            k = pAttr->speed / i;
            CLEAR_PERI_REG_MASK(SPI_CLOCK_REG(spiNum), SPI_CLK_EQU_SYSCLK);
            WRITE_PERI_REG(SPI_CLOCK_REG(spiNum),
                           (((i - 1) & SPI_CLKDIV_PRE) << SPI_CLKDIV_PRE_S) |
                           (((k - 1) & SPI_CLKCNT_N) << SPI_CLKCNT_N_S) |
                           ((((k + 1) / 2 - 1) & SPI_CLKCNT_H) << SPI_CLKCNT_H_S) |
                           (((k - 1) & SPI_CLKCNT_L) << SPI_CLKCNT_L_S)); //clear bit 31,set SPI clock div
        } else {
            WRITE_PERI_REG(SPI_CLOCK_REG(spiNum), SPI_CLK_EQU_SYSCLK); // 80Mhz speed
        }
        // Enable MOSI
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_CS_SETUP | SPI_CS_HOLD | SPI_USR_MOSI);

        // CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_CS_HOLD);/////////////add
        SET_PERI_REG_MASK(SPI_CTRL2_REG(spiNum), ((0x4 & SPI_MISO_DELAY_NUM) << SPI_MISO_DELAY_NUM_S)); //delay num

    } else if (SpiMode_Slave == pAttr->mode) {

        // SPI mode type
        SET_PERI_REG_MASK(SPI_SLAVE_REG(spiNum), SPI_SLAVE_MODE);
        // SPI mode type
        SET_PERI_REG_MASK(SPI_SLAVE_REG(spiNum), SPI_SLV_WR_RD_BUF_EN);
        // SPI Send buffer
        // SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MISO_HIGHPART);// By default slave send buffer C8-C15

        // If do not set delay cycles, slave not working,master cann't get the data.
        SET_PERI_REG_MASK(SPI_CTRL2_REG(spiNum), ((0x2 & SPI_MOSI_DELAY_NUM) << SPI_MOSI_DELAY_NUM_S)); //delay num
        // SPI Speed
        WRITE_PERI_REG(SPI_CLOCK_REG(spiNum), 0);

        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_CS_SETUP);/////////////add
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MOSI);

        // By default format::CMD(8bits)+ADDR(8bits)+DATA(32bytes)
        // set pAttr->cmdLen bit slave recieve command length
        // set 1 bytes status buffer length
        // set pAttr->addrLen bit slave recieve read address length
        // set pAttr->addrLen bit slave recieve write address length
        // set 32 bytes slave recieve buffer length
        SET_PERI_REG_BITS(SPI_USER2_REG(spiNum), SPI_USR_COMMAND_BITLEN,
                          (7), SPI_USR_COMMAND_BITLEN_S);
        SET_PERI_REG_BITS(SPI_SLAVE1_REG(spiNum), SPI_SLV_STATUS_BITLEN,
                          (7), SPI_SLV_STATUS_BITLEN_S);
        SET_PERI_REG_BITS(SPI_SLAVE1_REG(spiNum), SPI_SLV_WR_ADDR_BITLEN,
                          (7), SPI_SLV_WR_ADDR_BITLEN_S);
        SET_PERI_REG_BITS(SPI_SLAVE1_REG(spiNum), SPI_SLV_RD_ADDR_BITLEN,
                          (7), SPI_SLV_RD_ADDR_BITLEN_S);
        SET_PERI_REG_BITS(SPI_SLV_WRBUF_DLEN_REG(spiNum), SPI_SLV_WRBUF_DBITLEN,
                          (32 * 8 - 1), SPI_SLV_WRBUF_DBITLEN_S);
        SET_PERI_REG_BITS(SPI_SLV_RDBUF_DLEN_REG(spiNum), SPI_SLV_RDBUF_DBITLEN,
                          (32 * 8 - 1), SPI_SLV_RDBUF_DBITLEN_S);
    } else {
        // To do nothing
    }

    char i;
    for (i = 0; i < 16; ++i) {
        WRITE_PERI_REG((SPI_W0_REG(spiNum) + (i << 2)), 0);
    }
}

/**
 * @brief Set address value by master mode.
 *
 */
IRAM_ATTR void spi_master_cfg_addr(spi_num_e spiNum, uint32_t addr)
{
    if (spiNum > SpiNum_Max) {
        return;
    }
    // Set address
    SET_PERI_REG_BITS(SPI_ADDR_REG(spiNum), SPI_USR_ADDR_VALUE, addr, SPI_USR_ADDR_VALUE_S);
}

/**
 * @brief Set command value by master mode.
 *
 */
IRAM_ATTR void spi_master_cfg_cmd(spi_num_e spiNum, uint32_t cmd)
{
    if (spiNum > SpiNum_Max) {
        return;
    }
    // SPI_USER2 bit28-31 is cmd length,cmd bit length is value(0-15)+1,
    // bit15-0 is cmd value.
    SET_PERI_REG_BITS(SPI_USER2_REG(spiNum), SPI_USR_COMMAND_VALUE, cmd, SPI_USR_COMMAND_VALUE_S);
}

/**
 * @brief Send data to slave.
 *
 */
int  spi_master_send_data(spi_num_e spiNum, spi_data_t* pInData)
{
    char idx = 0;
    if ((spiNum > SpiNum_Max)
        || (NULL == pInData)
        || (64 < pInData->txDataLen)) {
        return -1;
    }
    uint32_t *value = pInData->txData;
    while (READ_PERI_REG(SPI_CMD_REG(spiNum))&SPI_USR);
    // Set command by user.
    if (pInData->cmdLen != 0) {
        // Max command length 16 bits.
        SET_PERI_REG_BITS(SPI_USER2_REG(spiNum), SPI_USR_COMMAND_BITLEN,
                          ((pInData->cmdLen << 3) - 1), SPI_USR_COMMAND_BITLEN_S);
        // Enable command
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_COMMAND);
        // Load command
        spi_master_cfg_cmd(spiNum, pInData->cmd);
    } else {
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_COMMAND);
        SET_PERI_REG_BITS(SPI_USER2_REG(spiNum), SPI_USR_COMMAND_BITLEN,
                          0, SPI_USR_COMMAND_BITLEN_S);
    }
    // Set Address by user.
    if (pInData->addrLen == 0) {
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_ADDR);
        SET_PERI_REG_BITS(SPI_USER1_REG(spiNum), SPI_USR_ADDR_BITLEN,
                          0, SPI_USR_ADDR_BITLEN_S);
    } else {
        if (NULL == pInData->addr) {
            return -1;
        }
        SET_PERI_REG_BITS(SPI_USER1_REG(spiNum), SPI_USR_ADDR_BITLEN,
                          ((pInData->addrLen << 3) - 1), SPI_USR_ADDR_BITLEN_S);
        // Enable address
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_ADDR);
        // Load address
        spi_master_cfg_addr(spiNum, *pInData->addr);
    }
    // Set data by user.
    if (pInData->txDataLen != 0) {
        if (NULL == value) {
            return -1;
        }
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MISO);
        // Enable MOSI
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MOSI);
        // Load send buffer
        do {
            WRITE_PERI_REG((SPI_W0_REG(spiNum) + (idx << 2)), *value++);
        } while (++idx < ((pInData->txDataLen / 4) + ((pInData->txDataLen % 4) ? 1 : 0)));

        // Set data send buffer length.Max data length 64 bytes.
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(spiNum), SPI_USR_MOSI_DBITLEN, ((pInData->txDataLen << 3) - 1), SPI_USR_MOSI_DBITLEN_S);

        SET_PERI_REG_BITS(SPI_MISO_DLEN_REG(spiNum), SPI_USR_MISO_DBITLEN, ((pInData->rxDataLen << 3) - 1), SPI_USR_MISO_DBITLEN_S);
    } else {
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MOSI);
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MISO);
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(spiNum), SPI_USR_MOSI_DBITLEN,
                          0, SPI_USR_MOSI_DBITLEN_S);

    }
    // Start send data
    SET_PERI_REG_MASK(SPI_CMD_REG(spiNum), SPI_USR);
    while (!(READ_PERI_REG(SPI_SLAVE_REG(spiNum))&SPI_TRANS_DONE));
    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(spiNum), SPI_TRANS_DONE);
    return 0;
}

/**
 * @brief Receive data from slave.
 *
 */
IRAM_ATTR int spi_master_recv_data(spi_num_e spiNum, spi_data_t* pData)
{
    char idx = 0;
    if ((spiNum > SpiNum_Max)
        || (NULL == pData)) {
        return -1;
    }
    uint32_t *value = pData->rxData;
    while (READ_PERI_REG(SPI_CMD_REG(spiNum))&SPI_USR);
    // Set command by user.
    if (pData->cmdLen != 0) {
        // Max command length 16 bits.
        SET_PERI_REG_BITS(SPI_USER2_REG(spiNum), SPI_USR_COMMAND_BITLEN,
                          ((pData->cmdLen << 3) - 1), SPI_USR_COMMAND_BITLEN_S);
        // Enable command
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_COMMAND);
        // Load command
        spi_master_cfg_cmd(spiNum, pData->cmd);
    } else {
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_COMMAND);
        SET_PERI_REG_BITS(SPI_USER2_REG(spiNum), SPI_USR_COMMAND_BITLEN,
                          0, SPI_USR_COMMAND_BITLEN_S);
    }
    // Set Address by user.
    if (pData->addrLen == 0) {
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_ADDR);
        SET_PERI_REG_BITS(SPI_USER1_REG(spiNum), SPI_USR_ADDR_BITLEN,
                          0, SPI_USR_ADDR_BITLEN_S);
    } else {
        if (NULL == pData->addr) {
            return -1;
        }
        SET_PERI_REG_BITS(SPI_USER1_REG(spiNum), SPI_USR_ADDR_BITLEN,
                          ((pData->addrLen << 3) - 1), SPI_USR_ADDR_BITLEN_S);
        // Enable address
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_ADDR);
        // Load address
        spi_master_cfg_addr(spiNum, *pData->addr);
    }
    // Set data by user.
    if (pData->rxDataLen != 0) {
        if (NULL == value) {
            return -1;
        }
        // Clear MOSI enable
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MOSI);
        // Enable MOSI
        SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MISO);
        // Set data send buffer length.Max data length 64 bytes.
        SET_PERI_REG_BITS(SPI_MISO_DLEN_REG(spiNum), SPI_USR_MISO_DBITLEN, ((pData->rxDataLen << 3) - 1), SPI_USR_MISO_DBITLEN_S);
    } else {
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MOSI);
        CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MISO);
        SET_PERI_REG_BITS(SPI_MISO_DLEN_REG(spiNum), SPI_USR_MISO_DBITLEN, 0, SPI_USR_MISO_DBITLEN_S);
    }
    // Start send data
    SET_PERI_REG_MASK(SPI_CMD_REG(spiNum), SPI_USR);

    while (READ_PERI_REG(SPI_CMD_REG(spiNum))&SPI_USR);
    // Read data out
    do {
        *value++ =  READ_PERI_REG(SPI_W0_REG(spiNum) + (idx << 2));
    } while (++idx < ((pData->rxDataLen / 4) + ((pData->rxDataLen % 4) ? 1 : 0)));

    return 0;
}


/**
 * @brief Send data to slave(ESP32,RD_STATUS or WR_STATUS).
 *
 */
void  spi_master_send_status(spi_num_e spiNum, uint8_t data)
{
    if (spiNum > SpiNum_Max) {
        return;
    }
    while (READ_PERI_REG(SPI_CMD_REG(spiNum))&SPI_USR);
    // enable MOSI
    SET_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MOSI);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(spiNum), SPI_USR_MISO | SPI_USR_DUMMY | SPI_USR_ADDR);

    // 8bits cmd, 0x04 is eps32 slave write cmd value
    WRITE_PERI_REG(SPI_USER2_REG(spiNum),
                   ((7 & SPI_USR_COMMAND_BITLEN) << SPI_USR_COMMAND_BITLEN_S)
                   | MASTER_WRITE_STATUS_TO_SLAVE_CMD);
    // Set data send buffer length.
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(spiNum), SPI_USR_MOSI_DBITLEN,
                      ((sizeof(data) << 3) - 1), SPI_USR_MOSI_DBITLEN_S);

    WRITE_PERI_REG(SPI_W0_REG(spiNum), (uint32_t)(data));
    // start SPI
    SET_PERI_REG_MASK(SPI_CMD_REG(spiNum), SPI_USR);
}

#ifdef __cplusplus
}
#endif
