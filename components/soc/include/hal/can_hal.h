// Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
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

/*******************************************************************************
 * NOTICE
 * The hal is not public api, don't use in application code.
 * See readme.md in soc/include/hal/readme.md
 ******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include "hal/can_types.h"
#include "hal/can_ll.h"

/* ------------------------- Defines and Typedefs --------------------------- */

#define CAN_HAL_SET_FLAG(var, flag)         ((var) |= (flag))
#define CAN_HAL_RESET_FLAG(var, flag)       ((var) &= ~(flag))

//HAL state flags
#define CAN_HAL_STATE_FLAG_RUNNING          (1 << 0)    //Controller is active (not in reset mode)
#define CAN_HAL_STATE_FLAG_RECOVERING       (1 << 1)    //Bus is undergoing bus recovery
#define CAN_HAL_STATE_FLAG_ERR_WARN         (1 << 2)    //TEC or REC is >= error warning limit
#define CAN_HAL_STATE_FLAG_ERR_PASSIVE      (1 << 3)    //TEC or REC is >= 128
#define CAN_HAL_STATE_FLAG_BUS_OFF          (1 << 4)    //Bus-off due to TEC >= 256
#define CAN_HAL_STATE_FLAG_TX_BUFF_OCCUPIED (1 << 5)    //Transmit buffer is occupied
#ifdef CAN_ERRATA_HW_RESET_PERIPH
#define CAN_HAL_STATE_FLAG_PERIPH_WAS_RESET (1 << 6)    //Peripheral was reset on entry to ISR due to errata
#define CAN_HAL_STATE_FLAG_TX_RETRY         (1 << 7)    //Peripheral reset has cancelled a pending TX that needs to be retried
#endif
#ifdef CAN_ERRATA_11_WORKAROUND
#define CAN_HAL_STATE_FLAG_TX_INTR_MISS     (1 << 8)    //Workaround has caused TX buffer free interrupt to be cleared
#endif

//Interrupt Events
#define CAN_HAL_EVENT_BUS_OFF               (1 << 0)    //Bus-off has occurred
#define CAN_HAL_EVENT_BUS_RECOV_CPLT        (1 << 1)    //Bus-off recovery has been completed
#define CAN_HAL_EVENT_BUS_RECOV_PROGRESS    (1 << 2)    //Bus-recovery in progress. TEC has dropped below error warning limit
#define CAN_HAL_EVENT_ABOVE_EWL             (1 << 3)    //TEC or REC surpassed error warning limit
#define CAN_HAL_EVENT_BELOW_EWL             (1 << 4)    //TEC and REC are both below error warning
#define CAN_HAL_EVENT_ERROR_PASSIVE         (1 << 5)    //Entered error passive
#define CAN_HAL_EVENT_ERROR_ACTIVE          (1 << 6)    //Returned to error active
#define CAN_HAL_EVENT_BUS_ERR               (1 << 7)    //A bus error has occurred
#define CAN_HAL_EVENT_ARB_LOST              (1 << 8)    //Arbitration was lost
#define CAN_HAL_EVENT_RX_BUFF_FRAME         (1 << 9)    //One or more frames are in the RX FIFO
#define CAN_HAL_EVENT_TX_BUFF_FREE          (1 << 10)   //TX buffer is free (transmission completed)
#ifdef CAN_ERRATA_HW_RESET_PERIPH
#define CAN_HAL_EVENT_ERRATA_NEED_RESET     (1 << 11)   //Errata has caused the peripheral to require a reset
#endif


typedef can_ll_frame_buffer_t can_hal_frame_t;

typedef struct {
    can_dev_t *dev;
    uint32_t state_flags;
#ifdef CAN_ERRATA_HW_RESET_PERIPH
    //Space to store state and configuration when resetting peripheral
    can_ll_reg_save_t reg_save;
    uint8_t alc_reg;
    uint8_t ecc_reg;
    uint8_t rx_msg_cnt_reg;
    can_hal_frame_t current_tx_frame;
#endif
} can_hal_context_t;

/* ---------------------------- Init and Config ----------------------------- */

/**
 * @brief Initialize CAN peripheral and HAL context
 *
 * Sets HAL context, puts CAN peripheral into reset mode, then sets some
 * registers with default values.
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successfully initialized, false otherwise.
 */
bool can_hal_init(can_hal_context_t *hal_ctx);

/**
 * @brief Deinitialize the CAN peripheral and HAL context
 *
 * Clears any unhandled interrupts and unsets HAL context
 *
 * @param hal_ctx Context of the HAL layer
 */
void can_hal_deinit(can_hal_context_t *hal_ctx);

/**
 * @brief Configure the CAN peripheral
 *
 * @param hal_ctx Context of the HAL layer
 * @param t_config Pointer to timing configuration structure
 * @param f_config Pointer to filter configuration structure
 * @param intr_mask Mask of interrupts to enable
 * @param clkout_divider Clock divider value for CLKOUT. Set to -1 to disable CLKOUT
 */
void can_hal_configure(can_hal_context_t *hal_ctx, const can_timing_config_t *t_config, const can_filter_config_t *f_config, uint32_t intr_mask, uint32_t clkout_divider);

/* -------------------------------- Actions --------------------------------- */

/**
 * @brief Start the CAN peripheral
 *
 * Start the CAN peripheral by configuring its operating mode, then exiting
 * reset mode so that the CAN peripheral can participate in bus activities.
 *
 * @param hal_ctx Context of the HAL layer
 * @param mode Operating mode
 * @return True if successfully started, false otherwise.
 */
bool can_hal_start(can_hal_context_t *hal_ctx, can_mode_t mode);

/**
 * @brief Stop the CAN peripheral
 *
 * Stop the CAN peripheral by entering reset mode to stop any bus activity, then
 * setting the operating mode to Listen Only so that REC is frozen.
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successfully stopped, false otherwise.
 */
bool can_hal_stop(can_hal_context_t *hal_ctx);

/**
 * @brief Start bus recovery
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successfully started bus recovery, false otherwise.
 */
static inline bool can_hal_start_bus_recovery(can_hal_context_t *hal_ctx)
{
    CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_RECOVERING);
    return can_ll_exit_reset_mode(hal_ctx->dev);
}

/**
 * @brief Get the value of the TX Error Counter
 *
 * @param hal_ctx Context of the HAL layer
 * @return TX Error Counter Value
 */
static inline uint32_t can_hal_get_tec(can_hal_context_t *hal_ctx)
{
    return can_ll_get_tec((hal_ctx)->dev);
}

/**
 * @brief Get the value of the RX Error Counter
 *
 * @param hal_ctx Context of the HAL layer
 * @return RX Error Counter Value
 */
static inline uint32_t can_hal_get_rec(can_hal_context_t *hal_ctx)
{
    return can_ll_get_rec((hal_ctx)->dev);
}

/**
 * @brief Get the RX message count register
 *
 * @param hal_ctx Context of the HAL layer
 * @return RX message count
 */
static inline uint32_t can_hal_get_rx_msg_count(can_hal_context_t *hal_ctx)
{
    return can_ll_get_rx_msg_count((hal_ctx)->dev);
}

/**
 * @brief Check if the last transmitted frame was successful
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successful
 */
static inline bool can_hal_check_last_tx_successful(can_hal_context_t *hal_ctx)
{
    return can_ll_is_last_tx_successful((hal_ctx)->dev);
}

/**
 * @brief Check if certain HAL state flags are set
 *
 * The HAL will maintain a record of the controller's state via a set of flags.
 * These flags are automatically maintained (i.e., set and reset) inside various
 * HAL function calls. This function checks if certain flags are currently set.
 *
 * @param hal_ctx Context of the HAL layer
 * @param check_flags Bit mask of flags to check
 * @return True if one or more of the flags in check_flags are set
 */
static inline bool can_hal_check_state_flags(can_hal_context_t *hal_ctx, uint32_t check_flags)
{
    return hal_ctx->state_flags & check_flags;
}

/* ----------------------------- Event Handling ----------------------------- */

/**
 * @brief Prepare for interrupt service routine
 *
 * This function should be the called at the beginning of an ISR. This
 * function will do the following:
 * - Read and clear interrupts
 * - Decode current events that triggered an interrupt
 * - Check if errata conditions require a reset.
 *
 * @param hal_ctx Context of the HAL layer
 * @return Bit mask of events that have occurred
 */
uint32_t can_hal_isr_entry(can_hal_context_t *hal_ctx);

/**
 * @brief Prepare to exit an interrupt service routine
 *
 * This function should be called at the end of an ISR. It will handle any
 * updating of the HAL's internal state
 *
 * @param hal_ctx Context of the HAL layer
 */
void can_hal_isr_exit(can_hal_context_t *hal_ctx);

/**
 * @brief Handle bus recovery complete
 *
 * This function should be called on an bus recovery complete event. It simply
 * enters reset mode to stop bus activity and resets the HAl state flags.
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successfully handled bus recovery completion, false otherwise.
 */
static inline bool can_hal_handle_bus_recov_cplt(can_hal_context_t *hal_ctx)
{
    return can_ll_enter_reset_mode((hal_ctx)->dev);
}

/**
 * @brief Handle arbitration lost
 *
 * This function should be called on an arbitration lost event. It simply clears
 * the clears the ALC register.
 *
 * @param hal_ctx Context of the HAL layer
 */
static inline void can_hal_handle_arb_lost(can_hal_context_t *hal_ctx)
{
#ifdef CAN_ERRATA_HW_RESET_PERIPH
    if (!(hal_ctx->state_flags & CAN_HAL_STATE_FLAG_PERIPH_WAS_RESET)) {
        (void) can_ll_get_arb_lost_cap((hal_ctx)->dev);
    }
#else
    (void) can_ll_get_arb_lost_cap((hal_ctx)->dev);
#endif
}

/**
 * @brief Handle bus error
 *
 * This function should be called on a bus error event. It reads and decodes the
 * details regarding the bus error stored in the ECC register. When the ECC
 * register is read, it is automatically cleared and the bus error interrupt
 * re-armed.
 *
 * @param hal_ctx Context of the HAL layer
 *
 */
static inline void can_hal_handle_bus_error(can_hal_context_t *hal_ctx)
{
#ifdef CAN_ERRATA_5_WORKAROUND
    if (!(hal_ctx->state_flags & CAN_HAL_STATE_FLAG_PERIPH_WAS_RESET)) {
        (void) can_ll_get_err_code_cap((hal_ctx)->dev);
    }
#else
    (void) can_ll_get_err_code_cap((hal_ctx)->dev);
#endif
}

/**
 * @brief Handle BUS OFF
 *
 * This function should be called on a BUS OFF event. Mode is set to LOM to
 * freeze both error counters. It's also possible that REC would have increased
 * before this handler is able to run. Therefore, set TEC to 0 to reset both TEC
 * and REC to 0, then reset TEC to 255 to retrigger the Bus OFF condition. Then
 * clear the re-triggered BUS OFF interrupt.
 *
 * @param hal_ctx Context of the HAL layer
 */
static inline void can_hal_handle_bus_off(can_hal_context_t *hal_ctx)
{
    can_ll_set_mode(hal_ctx->dev, CAN_MODE_LISTEN_ONLY);
    can_ll_set_tec(hal_ctx->dev, 0);
    can_ll_set_tec(hal_ctx->dev, 255);
    (void) can_ll_get_and_clear_intrs(hal_ctx->dev);    //Clear the re-triggered BUS OFF interrupt
}

/* ------------------------------- TX and RX -------------------------------- */

/**
 * @brief Format a CAN Frame
 *
 * This function takes a CAN message structure (containing ID, DLC, data, and
 * flags) and formats it to match the layout of the TX frame buffer.
 *
 * @param message Pointer to CAN message
 * @param frame Pointer to empty frame structure
 */
static inline void can_hal_format_frame(const can_message_t *message, can_hal_frame_t *frame)
{
    //Direct call to ll function
    can_ll_format_frame_buffer(message->identifier, message->data_length_code, message->data,
                               message->flags, frame);
}

/**
 * @brief Parse a CAN Frame
 *
 * This function takes a CAN frame (in the format of the RX frame buffer) and
 * parses it to a CAN message (containing ID, DLC, data and flags).
 *
 * @param frame Pointer to frame structure
 * @param message Pointer to empty message structure
 */
static inline void can_hal_parse_frame(can_hal_frame_t *frame, can_message_t *message)
{
    //Direct call to ll function
    can_ll_prase_frame_buffer(frame, &message->identifier, &message->data_length_code,
                              message->data, &message->flags);
}

/**
 * @brief Copy a frame into the TX buffer and transmit
 *
 * This function copies a formatted TX frame into the TX buffer, and the
 * transmit by setting the correct transmit command (e.g. normal, single shot,
 * self RX) in the command register.
 *
 * @param hal_ctx Context of the HAL layer
 * @param tx_frame Pointer to structure containing formatted TX frame
 */
void can_hal_set_tx_buffer_and_transmit(can_hal_context_t *hal_ctx, can_hal_frame_t *tx_frame);

/**
 * @brief Copy a frame from the RX buffer and release
 *
 * If the RX buffer points a valid frame (i.e., not overrun), this function
 * will copy the RX buffer, then release the buffer (so that it loads the next
 * frame in the RX FIFO). If the RX buffer points to an overrun frame, it indicates
 * that the RX FIFO contains one or more overurn frames which should be cleared.
 *
 * @param hal_ctx Context of the HAL layer
 * @param rx_frame Pointer to structure tostore RX frame
 * @return True if frame was copied and RX buffer released. False if RX buffer
 *         points to overrun frame.
 */
static inline bool can_hal_read_rx_buffer_and_clear(can_hal_context_t *hal_ctx, can_hal_frame_t *rx_frame)
{
    if (can_ll_get_status(hal_ctx->dev) & CAN_LL_STATUS_DOS) {
        //RX buffer points to an overrun frame in the RX FIFO
        return false;
    }
    can_ll_get_rx_buffer(hal_ctx->dev, rx_frame);
    can_ll_set_cmd_release_rx_buffer(hal_ctx->dev);
    return true;
}

/**
 * @brief Clear the RX FIFO of all overrun frames
 *
 * Clears all overrun frames from the RX FIFO.
 *
 * @param hal_ctx Context of the HAL layer
 * @return The number of overrun frames cleared from RX FIFO
 */
static inline uint32_t can_hal_clear_rx_fifo_overrun(can_hal_context_t *hal_ctx)
{
    uint32_t msg_count = can_ll_get_rx_msg_count(hal_ctx->dev);
    //Release RX buffer "msg_count" iterations so that all overurn frames are cleared
    for (int i = 0; i < msg_count; i++) {
        can_ll_set_cmd_release_rx_buffer(hal_ctx->dev);
    }
    can_ll_set_cmd_clear_data_overrun(hal_ctx->dev);
    return msg_count;
}

/* --------------------------- Errata Workaround ---------------------------- */

#ifdef CAN_ERRATA_HW_RESET_PERIPH
/**
 * @brief Prepare for a hardware reset
 *
 * Prepare for a hardware reset by saving stopping the controller and saving
 * registers. Hardware reset is required for errata workarounds.
 *
 * @param hal_ctx Context of the HAL layer
 */
void can_hal_prepare_for_hw_reset(can_hal_context_t *hal_ctx, uint32_t events);

/**
 * @brief Recover from a hardware reset
 *
 * Recover from a hardware reset by restoring registers, restarting the
 * controller, and resuming an transmission halted by the reset.
 *
 * @param hal_ctx Context of the HAL layer
 */
void can_hal_recover_from_hw_reset(can_hal_context_t *hal_ctx);
#endif

#ifdef CAN_ERRATA_11_WORKAROUND
/**
 * @brief Errata 11 workaround
 *
 * Call this function to workaround errata 11. This function will enter then
 * exit reset mode in order. Any ongoing transmission will be restarted.
 *
 * @param hal_ctx Context of the HAL layer
 */
void can_hal_errata_11_workaround(can_hal_context_t *hal_ctx);
#endif

#ifdef __cplusplus
}
#endif
