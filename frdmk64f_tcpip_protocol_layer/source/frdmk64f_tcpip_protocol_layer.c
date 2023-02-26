/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    frdmk64f_tcpip_protocol_layer.c
 * @brief   Application entry point.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "peripherals.h"
#include "MK64F12.h"
#include "pin_mux.h"
#include "board.h"
#include <stdio.h>

#include "lwip/opt.h"
#include "my_layer.h"

#if LWIP_NETCONN

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Stack size of the temporary lwIP initialization thread */
#define INIT_THREAD_STACKSIZE 512

/*! @brief Priority of the temporary lwIP initialization thread */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Handlers
 ******************************************************************************/

/*******************************************************************************
 * Functions
 ******************************************************************************/
/*!
 * @brief Initializes lwIP stack.
 *
 * @param arg unused
 */
static void stack_init (void *arg)
{
    LWIP_UNUSED_ARG (arg);

    my_layer_init ( );

    vTaskDelete (NULL);
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * @brief   Application entry point.
 */
int main (void)
{
	SYSMPU_Type *base = SYSMPU;

    /* Initialize board hardware */
    BOARD_InitBootPins ( );
    BOARD_InitBootClocks ( );
    BOARD_InitBootPeripherals ( );

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Initialize FSL debug console. */
    BOARD_InitDebugConsole ( );
#endif

    /* Print a note to terminal. */
    PRINTF("Practica 1: TCP/IP Protocol Layer\n\r");

    /* Disable SYSMPU. */
    base -> CESR &= ~SYSMPU_CESR_VLD_MASK;

    /* Initialize lwIP from thread */
    if (sys_thread_new ("main", stack_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }

    vTaskStartScheduler ( );

    /* Enter an infinite loop. */
    while (1)
    {

    }
    return 0 ;
}

#endif
