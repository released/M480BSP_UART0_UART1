/****************************************************************************
 * @file     sysclock.c
 * @version  V1.01
 * @Date     2019/03/06-10:02:20
 * @brief    NuMicro generated code file
 *
 * Copyright (C) 2013-2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

/********************
MCU: M487JIDAE(LQFP144)
Base Clocks:
LIRC: 10kHz
HIRC: 12MHz
LXT: 32.7680kHz
HXT: 12MHz
PLL: 192MHz
HSUSB_OTG_PHY: 30MHz
HCLK: 192MHz
PCLK0: 96MHz
PCLK1: 96MHz
Enabled-Module Frequencies:
ISP=Bus Clock (HCLK): 192MHz/Engine Clock: 12MHz
PDMA=Bus Clock (HCLK): 192MHz
TMR0=Bus Clock (PCLK0): 96MHz/Engine Clock: 12MHz
UART0=Bus Clock (PCLK0): 96MHz/Engine Clock: 12MHz
********************/

#include "M480.h"

/*----------------------------------------------------------------------------
  Define HXT clock.
  Please locate and modify the real one in your project.
  Otherwise, the project may fail to build.
 *----------------------------------------------------------------------------*/
#define __HXT         (12000000UL)  /*!< High Speed External Crystal Clock Frequency */

/*
 * @brief This function updates clock registers to fulfil the configuration
 * @param None
 * @return None
 */
void SYS_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    //CLK->PWRCTL = (CLK->PWRCTL & ~(0x0000000Ful)) | 0x0000001Ful;
    //CLK->PLLCTL = (CLK->PLLCTL & ~(0x108FFFFFul)) | 0x0008421Eul;
    //CLK->CLKDIV0 = (CLK->CLKDIV0 & ~(0xFFFFFFFFul)) | 0x00000000ul;
    //CLK->CLKDIV1 = (CLK->CLKDIV1 & ~(0x00FFFFFFul)) | 0x00000000ul;
    //CLK->CLKDIV3 = (CLK->CLKDIV3 & ~(0xFFFF0000ul)) | 0x00000000ul;
    //CLK->CLKDIV4 = (CLK->CLKDIV4 & ~(0x0000FFFFul)) | 0x00000000ul;
    //CLK->PCLKDIV = (CLK->PCLKDIV & ~(0x00000077ul)) | 0x00000011ul;
    //CLK->CLKSEL0 = (CLK->CLKSEL0 & ~(0x00F0003Ful)) | 0x00F3013Aul;
    //CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(0xFF777703ul)) | 0xBC777003ul;
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~(0x00003FFFul)) | 0x000003ABul;
    //CLK->CLKSEL3 = (CLK->CLKSEL3 & ~(0xFF03023Ful)) | 0xFF00003Ful;
    //CLK->AHBCLK = (CLK->AHBCLK & ~(0x0003D4EEul)) | 0x00008006ul;
    //CLK->APBCLK0 = (CLK->APBCLK0 & ~(0x7F3FF7FFul)) | 0x00010004ul;
    //CLK->APBCLK1 = (CLK->APBCLK1 & ~(0x4CCF1347ul)) | 0x00000000ul;
    //CLK->CLKOCTL = (CLK->CLKOCTL & ~(0x0000007Ful)) | 0x00000000ul;
    //SysTick->CTRL = (SysTick->CTRL & ~(0x00000005ul)) | 0x00000000ul;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk|CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk|CLK_STATUS_HXTSTB_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x108FFFFFul)) | 0x0008421Eul;

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* If the defines do not exist in your project, please refer to the related clk.h in the clk_h folder appended to the tool package. */
    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2);

    /* Enable IP clock */
    CLK_EnableModuleClock(ISP_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Set IP clock */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();

    return;
}
/*** (C) COPYRIGHT 2013-2019 Nuvoton Technology Corp. ***/
