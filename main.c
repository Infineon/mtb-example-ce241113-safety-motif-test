/*******************************************************************************
* File Name:   main.c
*
* Description: This project demonstrate the Class-B Safety test for MOTIF block
* of Infineon PSOC Control C3 MCU using ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"
#include "SelfTest.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* UART timeout in milliseconds */
#define UART_TIMEOUT_MS (10u)

/* Available commands */
#define SELFTEST_CMD_MOTIF_QD               ('1')
#define SELFTEST_CMD_MOTIF_QD_ERRORINJ      ('2')

/*
 * Peripheral frequency = 240MHz, Resolution = 4.166666667 ns
 * Emulated signal Period = 600 = (Period count *2)/(Peripheral frequency) = 5us
 * Duration of two     Q-Clk ticks = (1.25)us
 * Timer ticks between two Q-Clk ticks = (1.25*1000/4.166666667) = 300
 */
#define QCLK_REF_COUNT             (300u)
#define QCLK_REF_ERROR_MARGIN      (30u) //10% of total ticks
#define STARTING_DELAY             (1U)  //1 ms delay

/*******************************************************************************
* Global Variables
*******************************************************************************/
/*Motif self test configuration handler */
stl_motif_cfg_handle_t cfgObj =
{
    .motif_base     = MOTIF0_HW,
    .motif_config   = &MOTIF0_quaddec_config,

    /*TCPWM configuration to capture the time between two q-clk edge*/
    .qclk_base      = TCPWM0,
    .qclk           =
    {
        .idx = Tick_Time_NUM,
        .cfg = &Tick_Time_config,
    },

    /*TCPWM configuration for Phase-A,Phase-Band index input*/
    .sgen_base      = {TCPWM0, TCPWM0, TCPWM0},
    .sgen           =
    {
        {
            .idx = EMU_PH_A_NUM,
            .cfg = &EMU_PH_A_config,
        },
        {
            .idx = EMU_PH_B_NUM,
            .cfg = &EMU_PH_B_config,
        },
        {
            .idx = EMU_PH_I_NUM,
            .cfg = &EMU_PH_I_config,
        },
    },

    .ref_count = QCLK_REF_COUNT,
    .margin_count = QCLK_REF_ERROR_MARGIN,
    .delay = STARTING_DELAY
};

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj;           /** Debug UART HAL object  */


/* Variable to store the input command from UART*/
uint8_t cmd;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* MOTIF test functions in quadrature decoder mode */
void motif_qd_test(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It performs Class-B safety test for TCPMW block.
* SelfTest is performed for Timer/Counter, PWM and PWM gate Kill based on the
* user command.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*Debug UART enable*/
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initializes redirecting of low-level I/O. */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "CLass-B: MOTIF SAFETY TEST "
           "****************** \r\n\n");

    /* Display available commands */
    printf("Available commands \r\n");
    printf("1 : Run MOTIF SelfTest in Quadrature Decoder mode\r\n");
    printf("2 : Run MOTIF SelfTest after injecting error \r\n");

    for (;;)
    {
        /* Storing the data send from the terminal through UART in cmd */
        result = mtb_hal_uart_get(&DEBUG_UART_hal_obj, &cmd, UART_TIMEOUT_MS);

        /* check for timeout if data is not received*/
        if (result != MTB_HAL_UART_RSLT_ERR_CSP_UART_GETC_TIMEOUT)
        {
            if (SELFTEST_CMD_MOTIF_QD == cmd)
            {
                printf("\r\n[Command] : Run MOTIF SelfTest in Quadrature Decoder mode\r\n");
                motif_qd_test();
            }
            else if (SELFTEST_CMD_MOTIF_QD_ERRORINJ == cmd)
            {
                printf("\r\n[Command] : Run MOTIF SelfTest with injected error.\r\n");
                cfgObj.ref_count = (QCLK_REF_COUNT*2u);
                motif_qd_test();
            }
            else
            {
                printf("\r\nEnter a valid command\r\n");
            }
        }

    }
}

/*******************************************************************************
* Function Name: motif_qd_test
********************************************************************************
* Summary:
*
*
* Parameters:
*  none
*
* Return :
*  void
*
*******************************************************************************/
void motif_qd_test(void)
{
    /* Initialize the MOTIF for Self Test. */
    SelfTest_Motif_Init(&cfgObj);

    /* Run MOTIF Self Test Self Test... */
    if (OK_STATUS != SelfTest_Motif_Start(&cfgObj))
    {
        /* Process error */
        printf("Error: MOTIF Self Test in QD Mode fail\r\n");
    }
    else
    {
        printf("Success: MOTIF Self Test in QD Mode pass\r\n");
    }
}

/* [] END OF FILE */

