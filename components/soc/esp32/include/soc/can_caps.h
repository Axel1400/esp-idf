// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if (CONFIG_ESP32_REV_MIN >= 2)
#define CAN_BRP_DIV_SUPPORTED       1
#define CAN_BRP_DIV_THRESH          128
//Any even number from 2 to 128, or multiples of 4 from 132 to 256
#define CAN_BRP_IS_VALID(brp)       (((brp) >= 2 && (brp) <= 128 && ((brp) & 0x1) == 0) || ((brp) >= 132 && (brp) <= 256 && ((brp) & 0x3) == 0))
#else
//Any even number from 2 to 128
#define CAN_BRP_IS_VALID(brp)       ((brp) >= 2 && (brp) <= 128 && ((brp) & 0x1) == 0)
#endif

/* -------------------------- Errata Workarounds ---------------------------- */

/**
 * Errata: When a transmit interrupt occurs, and interrupt register is read on
 * the same clock cycle, the transmit interrupt could be lost.
 * Workaround: Check the STATUS_TRANSMIT_BUFFER bit each time the INTERRUPT_REG is read
 */
#define CAN_ERRATA_4_WORKAROUND             1

/**
 * Errata: Receiving an erroneous data frame can cause the data bytes of the next
 * received data frame to be invalid.
 * Workaround: Reset peripheral
 */
#define CAN_ERRATA_5_WORKAROUND             1

/**
 * Errata: When RX FIFO overruns and RX message counter maxes out (at 64 messages),
 * any messages read from the RX buffer will be corrupt
 * Workaround: Reset peripheral.
 * 
 * Workaround threshold set at 63 to prevent failure of detecting errata condition
 */
#define CAN_ERRATA_10_WORKAROUND            1
#define CAN_ERRATA_10_WORKAROUND_THRESHOLD  63

/**
 * Errata: When the release buffer command is set, and at the same APB clock cycle
 * an overrun byte is written into the RX FIFO, the RX FIFO's internal byte counter
 * will not be decremented by the release buffer command.
 * Workaround: Enter and exit reset mode
 */
#define CAN_ERRATA_11_WORKAROUND            1

/**
 * Some errata workarounds will require a hardware reset of the peripheral, thus
 * requires the peripheral's registers to be saved and restored.
 */
#if defined(CAN_ERRATA_10_WORKAROUND)
#define CAN_ERRATA_HW_RESET_PERIPH          1
#endif

//Todo: Add ECC decode capabilities
//Todo: Add ALC decode capability

#ifdef __cplusplus
}
#endif
