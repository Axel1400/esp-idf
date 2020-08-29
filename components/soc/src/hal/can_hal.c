// Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
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

#include <stddef.h>
#include <string.h>
#include <assert.h>
#include "hal/can_hal.h"

//Default values written to various registers on initialization
#define CAN_HAL_INIT_TEC    0
#define CAN_HAL_INIT_REC    0
#define CAN_HAL_INIT_EWL    96

/* ---------------------------- Init and Config ----------------------------- */

bool can_hal_init(can_hal_context_t *hal_ctx)
{
    //Initialize HAL context
    hal_ctx->dev = &CAN;
    hal_ctx->state_flags = 0;
    //Initialize CAN controller, and set default values to registers
    if (!can_ll_enter_reset_mode(hal_ctx->dev)) {    //Must enter reset mode to write to config registers
        return false;
    }
    can_ll_enable_extended_reg_layout(hal_ctx->dev);        //Set PeliCAN address layout
    can_ll_set_mode(hal_ctx->dev, CAN_MODE_LISTEN_ONLY);    //Freeze REC by changing to LOM mode
    //Both TEC and REC should start at 0
    can_ll_set_tec(hal_ctx->dev, CAN_HAL_INIT_TEC);
    can_ll_set_rec(hal_ctx->dev, CAN_HAL_INIT_REC);
    can_ll_set_err_warn_lim(hal_ctx->dev, CAN_HAL_INIT_EWL);    //Set default value of for EWL
    return true;
}

void can_hal_deinit(can_hal_context_t *hal_ctx)
{
    //Clear any pending registers
    (void) can_ll_get_and_clear_intrs(hal_ctx->dev);
    can_ll_set_enabled_intrs(hal_ctx->dev, 0);
    (void) can_ll_get_arb_lost_cap(hal_ctx->dev);
    (void) can_ll_get_err_code_cap(hal_ctx->dev);
    hal_ctx->dev = NULL;
}

void can_hal_configure(can_hal_context_t *hal_ctx, const can_timing_config_t *t_config, const can_filter_config_t *f_config, uint32_t intr_mask, uint32_t clkout_divider)
{
    //Configure bus timing, acceptance filter, CLKOUT, and interrupts
    can_ll_set_bus_timing(hal_ctx->dev, t_config->brp, t_config->sjw, t_config->tseg_1, t_config->tseg_2, t_config->triple_sampling);
    can_ll_set_acc_filter(hal_ctx->dev, f_config->acceptance_code, f_config->acceptance_mask, f_config->single_filter);
    can_ll_set_clkout(hal_ctx->dev, clkout_divider);
    can_ll_set_enabled_intrs(hal_ctx->dev, intr_mask);
    (void) can_ll_get_and_clear_intrs(hal_ctx->dev);    //Clear any latched interrupts
}

/* -------------------------------- Actions --------------------------------- */

bool can_hal_start(can_hal_context_t *hal_ctx, can_mode_t mode)
{
    can_ll_set_mode(hal_ctx->dev, mode);                //Set operating mode
    (void) can_ll_get_and_clear_intrs(hal_ctx->dev);    //Clear any latched interrupts
    CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_RUNNING);
    can_ll_set_cmd_clear_data_overrun(hal_ctx->dev);
    return can_ll_exit_reset_mode(hal_ctx->dev);        //Return false if failed to exit reset mode
}

bool can_hal_stop(can_hal_context_t *hal_ctx)
{
    if (!can_ll_enter_reset_mode(hal_ctx->dev)) {
        return false;
    }
    (void) can_ll_get_and_clear_intrs(hal_ctx->dev);
    can_ll_set_mode(hal_ctx->dev, CAN_MODE_LISTEN_ONLY);    //Freeze REC by changing to LOM mode
    //Any TX is immediately halted on entering reset mode
    CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
    CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_RUNNING);
    return true;
}

/* ----------------------------- Event Handling ----------------------------- */

uint32_t can_hal_isr_entry(can_hal_context_t *hal_ctx)
{
    uint32_t events = 0;
    //Read interrupt, status
    uint32_t interrupts = can_ll_get_and_clear_intrs(hal_ctx->dev);
    uint32_t status = can_ll_get_status(hal_ctx->dev);
    uint32_t tec = can_ll_get_tec(hal_ctx->dev);
    uint32_t rec = can_ll_get_rec(hal_ctx->dev);
    uint32_t rx_msg_count = can_ll_get_rx_msg_count(hal_ctx->dev);

    //Receive Interrupt set whenever RX FIFO  is not empty
    if (interrupts & CAN_LL_INTR_RI) {
#ifdef CAN_ERRATA_10_WORKAROUND     //Check for errata 10 condition
        if (rx_msg_count >= CAN_ERRATA_10_WORKAROUND_THRESHOLD) {
            events |= CAN_HAL_EVENT_ERRATA_NEED_RESET;
        }
#endif
        events |= CAN_HAL_EVENT_RX_BUFF_FRAME;
    }
    //Transmit interrupt set whenever TX buffer becomes free
#ifdef CAN_ERRATA_4_WORKAROUND      //Check for errata 4 condition
    if ((status & CAN_LL_STATUS_TBS) && ((interrupts & CAN_LL_INTR_TI) || (hal_ctx->state_flags & CAN_HAL_STATE_FLAG_TX_BUFF_OCCUPIED))) {
#else
    if (interrupts & CAN_LL_INTR_TI) {
#endif
        events |= CAN_HAL_EVENT_TX_BUFF_FREE;
        CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
    }
    //Error Warning Interrupt set whenever Error or Bus Status bit changes
    if (interrupts & CAN_LL_INTR_EI) {
        if (status & CAN_LL_STATUS_BS) {
            //Currently in BUS OFF state
            if (status & CAN_LL_STATUS_ES) {    //EWL is exceeded, thus must have entered BUS OFF
                events |= CAN_HAL_EVENT_BUS_OFF;
                //Any TX would have been halted by entering bus off. Reset its flag
                CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
                CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_BUS_OFF);
            } else {
                //Below EWL. Therefore TEC is counting down in bus recovery
                events |= CAN_HAL_EVENT_BUS_RECOV_PROGRESS;
            }
        } else {
            //Not in BUS OFF
            if (status & CAN_LL_STATUS_ES) {
                events |= CAN_HAL_EVENT_ABOVE_EWL;  //Just Exceeded EWL
                CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_ERR_WARN);
            } else if (hal_ctx->state_flags & CAN_HAL_STATE_FLAG_RECOVERING) {
                //Previously undergoing bus recovery
                events |= CAN_HAL_EVENT_BUS_RECOV_CPLT;
                CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_RECOVERING);
                CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_BUS_OFF);
            } else {
                events |= CAN_HAL_EVENT_BELOW_EWL;
                CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_ERR_WARN);
            }
        }
    }
    //Error Passive Interrupt on transition from error active to passive or vice versa
    if (interrupts & CAN_LL_INTR_EPI) {
        if (tec >= CAN_ERR_PASS_THRESH || rec >= CAN_ERR_PASS_THRESH) {
            events |= CAN_HAL_EVENT_ERROR_PASSIVE;
            CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_ERR_PASSIVE);
        } else {
            events |= CAN_HAL_EVENT_ERROR_ACTIVE;
            CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_ERR_PASSIVE);
        }
    }
    //Arbitration Lost Interrupt triggered on losing arbitration
    if (interrupts & CAN_LL_INTR_ALI) {
        events |= CAN_HAL_EVENT_ARB_LOST;
    }
    //Bus error interrupt triggered on a bus error (e.g. bit, ACK, stuff etc)
    if (interrupts & CAN_LL_INTR_BEI) {
#ifdef CAN_ERRATA_5_WORKAROUND      //Check for errata 5 condition
        can_ll_err_type_t type;
        can_ll_err_dir_t dir;
        can_ll_err_seg_t seg;
        uint32_t ecc = can_ll_get_err_code_cap(hal_ctx->dev);
        can_ll_decode_err_code_cap(ecc, &type, &dir, &seg);
        if (seg == CAN_LL_ERR_SEG_DATA || seg == CAN_LL_ERR_SEG_CRC_SEQ || seg == CAN_LL_ERR_SEG_ACK_DELIM) {
            events |= CAN_HAL_EVENT_ERRATA_NEED_RESET;
        }
        hal_ctx->ecc_reg = (uint8_t)ecc;
#endif
        events |= CAN_HAL_EVENT_BUS_ERR;
    }

    return events;
}

void can_hal_isr_exit(can_hal_context_t *hal_ctx)
{
    CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_PERIPH_WAS_RESET);
    CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_TX_INTR_MISS);
}

/* ------------------------------- TX and RX -------------------------------- */

void can_hal_set_tx_buffer_and_transmit(can_hal_context_t *hal_ctx, can_hal_frame_t *tx_frame)
{
    //Copy frame into tx buffer
    can_ll_set_tx_buffer(hal_ctx->dev, tx_frame);
    //Hit the send command
    if (tx_frame->self_reception) {
        if (tx_frame->single_shot) {
            can_ll_set_cmd_self_rx_single_shot(hal_ctx->dev);
        } else {
            can_ll_set_cmd_self_rx_request(hal_ctx->dev);
        }
    } else if (tx_frame->single_shot){
        can_ll_set_cmd_tx_single_shot(hal_ctx->dev);
    } else {
        can_ll_set_cmd_tx(hal_ctx->dev);
    }
#ifdef CAN_ERRATA_HW_RESET_PERIPH
    //Save current TX frame in case peripheral is reset at any time
    memcpy(&hal_ctx->current_tx_frame, tx_frame, sizeof(can_hal_frame_t));
#endif
    CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
}

/* --------------------------- Errata Workaround ---------------------------- */

#ifdef CAN_ERRATA_HW_RESET_PERIPH
void can_hal_prepare_for_hw_reset(can_hal_context_t *hal_ctx, uint32_t events)
{
    //Check if a on going transmission will be cancelled by hardware reset
    if (!(events & CAN_HAL_EVENT_TX_BUFF_FREE) && !(can_ll_get_status(hal_ctx->dev) & CAN_LL_STATUS_TBS)) {
        CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_TX_RETRY);
    }
    //Some register must read and saved before enetering reset mode
    hal_ctx->alc_reg = (uint8_t) can_ll_get_arb_lost_cap(hal_ctx->dev);
    hal_ctx->rx_msg_cnt_reg = (uint8_t) can_ll_get_rx_msg_count(hal_ctx->dev);
#ifndef CAN_ERRATA_5_WORKAROUND
    //Errata 5 workaround would already have read this register
    hal_ctx->ecc_reg = (uint8_t) can_ll_get_err_code_cap(hal_ctx->dev);
#endif
    //Enter reset mode to stop controller.
    can_ll_enter_reset_mode(hal_ctx->dev);
    //Save remaining registers after stopping
    can_ll_save_reg(hal_ctx->dev, &hal_ctx->reg_save);
}

void can_hal_recover_from_hw_reset(can_hal_context_t *hal_ctx)
{
    can_ll_enter_reset_mode(hal_ctx->dev);
    can_ll_enable_extended_reg_layout(hal_ctx->dev);
    can_ll_restore_reg(hal_ctx->dev, &hal_ctx->reg_save);
    can_ll_exit_reset_mode(hal_ctx->dev);
    (void) can_ll_get_and_clear_intrs(hal_ctx->dev);    //Clear any latched interrupts
    if (hal_ctx->state_flags & CAN_HAL_STATE_FLAG_TX_RETRY) {
        //Retry cancelled transmissions due to the reset
        can_hal_set_tx_buffer_and_transmit(hal_ctx, &hal_ctx->current_tx_frame);
        CAN_HAL_RESET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_TX_RETRY);
    }
    CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_PERIPH_WAS_RESET);
}
#endif

#ifdef CAN_ERRATA_11_WORKAROUND
void can_hal_errata_11_workaround(can_hal_context_t *hal_ctx)
{
    bool retry_tx = false;
    if (can_hal_check_state_flags(hal_ctx, CAN_HAL_STATE_FLAG_TX_BUFF_OCCUPIED)) {
        if (can_ll_get_status(hal_ctx->dev) & CAN_LL_STATUS_TBS) {
            //If TX completed betwen ISR entry and now, entering reset mode will clear
            //that interrupt. So we set a flag here.
            CAN_HAL_SET_FLAG(hal_ctx->state_flags, CAN_HAL_STATE_FLAG_TX_INTR_MISS);
        } else {
            retry_tx = true;
        }
    }
    can_ll_enter_reset_mode(hal_ctx->dev);
    can_ll_exit_reset_mode(hal_ctx->dev);
    if (retry_tx) {
        can_hal_set_tx_buffer_and_transmit(hal_ctx, &hal_ctx->current_tx_frame);
    }
}
#endif