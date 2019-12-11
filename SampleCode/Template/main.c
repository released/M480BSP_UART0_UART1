/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#define PLL_CLOCK           		192000000

#define BUF_LEN					(1024)

#define FIFO_THRESHOLD 			4
#define RX_BUFFER_SIZE 			128
#define RX_TIMEOUT_CNT 			60 //40~255

#define RX_UNKNOWN_LENGTH

#if defined (RX_UNKNOWN_LENGTH)
#define UART_RX_IDEL(uart) (((uart)->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk )>> UART_FIFOSTS_RXIDLE_Pos)

enum
{
    eUART_RX_Received_Data_Finish = 0,
    eUART_RX_Received_Data_NOT_Finish
};

volatile uint8_t g_au8UART0_RX_Buffer[RX_BUFFER_SIZE] = {0}; // UART Rx received data Buffer (RAM)
volatile uint8_t g_bUART0_RX_Received_Data_State = eUART_RX_Received_Data_NOT_Finish;
volatile uint8_t g_u8UART0_RDA_Trigger_Cnt = 0; // UART RDA interrupt trigger times counter
volatile uint8_t g_u8UART0_RXTO_Trigger_Cnt = 0; // UART RXTO interrupt trigger times counter

#endif

typedef struct {
	uint8_t buf[BUF_LEN];
	uint16_t len;
	uint8_t end;
}UART_BUF_t;

UART_BUF_t uart0Dev;

extern void SYS_Init(void); 

void send_UART1String(uint8_t* Data)
{
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		UART_WRITE(UART1,Data[i++]);	
	}
}

void TMR0_IRQHandler(void)
{
//	static uint16_t cnt_gpio = 0;

	static uint32_t LOG = 0;
	static uint16_t CNT = 0;

	uint8_t buffer[16] = {0};

    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

		if (CNT++ >= 1000)
		{
			CNT = 0;
        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);

			sprintf((char*)buffer ,"LOG:%d", LOG);
//			UART_Write(UART1,buffer,strlen((char*)buffer));
			send_UART1String(buffer);
		}
    }
}

void TIMER0_Init(void)
{
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);	
    TIMER_Start(TIMER0);
}

void GPIO_Init(void)
{
    GPIO_SetMode(PH, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT2, GPIO_MODE_OUTPUT);	
}


void UART1_IRQHandler(void)
{
	uint8_t res = 0;
	if (UART_GET_INT_FLAG(UART1,UART_INTSTS_RDAINT_Msk))
	{
		while(!UART_GET_RX_EMPTY(UART1))
		{
			res = UART_READ(UART1);
			printf("%s (RDAINT) : %2d\r\n" , __FUNCTION__ , res);
		}
	}
	
	if (UART_GET_INT_FLAG(UART1,UART_INTSTS_RXTOIF_Msk))
	{
		while(!UART_GET_RX_EMPTY(UART1))
		{
			res = UART_READ(UART1);
			printf("%s (RXTOIF) : %2d \r\n" , __FUNCTION__ , res);			
		}
	}
	
}

void UART1_Init(void)	//PB2 , PB3
{
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    SYS_ResetModule(UART1_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART1, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART1, RX_TIMEOUT_CNT);

	UART1->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART1->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART1, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);	
	NVIC_EnableIRQ(UART1_IRQn);
	UART_WAIT_TX_EMPTY(UART1);	
}

void UART0_IRQHandler(void)
{
#if defined (RX_UNKNOWN_LENGTH)
	uint8_t i;
	static uint16_t u16UART_RX_Buffer_Index = 0;

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))    
    {
        /* UART receive data available flag */
        
        /* Record RDA interrupt trigger times */
        g_u8UART0_RDA_Trigger_Cnt++;
        
        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            g_au8UART0_RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE) 
                u16UART_RX_Buffer_Index = 0;
        }
    }
    else if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RXTOINT_Msk)) 
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        g_u8UART0_RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            g_au8UART0_RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */
        UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        u16UART_RX_Buffer_Index = 0;
        g_bUART0_RX_Received_Data_State = eUART_RX_Received_Data_Finish;
    }
#else

	if (UART_GET_INT_FLAG(UART0,UART_INTSTS_RDAINT_Msk))
	{
		while(!UART_GET_RX_EMPTY(UART0))
		{
			uart0Dev.buf[uart0Dev.len++] = UART_READ(UART0);
		}
	}
	
	if (UART_GET_INT_FLAG(UART0,UART_INTSTS_RXTOIF_Msk))
	{
		while(!UART_GET_RX_EMPTY(UART0))
		{
			uart0Dev.buf[uart0Dev.len++] = UART_READ(UART0);
		}
		
		uart0Dev.end = 1;
	}
#endif
	
}

void UART0_Process(void)
{
	/*
		EC_M451_UART_Timerout_V1.00.zip
		https://www.nuvoton.com/hq/resource-download.jsp?tp_GUID=EC0120160728090754
	*/

	
	#if defined (RX_UNKNOWN_LENGTH)
        /* Wait to receive UART data */
        while(UART_RX_IDEL(UART0));

        /* Start to received UART data */
        g_bUART0_RX_Received_Data_State = eUART_RX_Received_Data_NOT_Finish;        
        /* Wait for receiving UART message finished */
        while(g_bUART0_RX_Received_Data_State != eUART_RX_Received_Data_Finish); 

        printf("\nUART0 Rx Received Data : %s\n",g_au8UART0_RX_Buffer);
        printf("UART0 Rx RDA (Fifofull) interrupt times : %d\n",g_u8UART0_RDA_Trigger_Cnt);
        printf("UART0 Rx RXTO (Timeout) interrupt times : %d\n",g_u8UART0_RXTO_Trigger_Cnt);

        /* Reset UART interrupt parameter */
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        g_u8UART0_RDA_Trigger_Cnt = 0; // UART RDA interrupt times
        g_u8UART0_RXTO_Trigger_Cnt = 0; // UART RXTO interrupt times
	#else
	if (uart0Dev.end)
	{
		while(!UART_GET_RX_EMPTY(UART0))
		{
			uart0Dev.buf[uart0Dev.len++] = UART_READ(UART0);
		}

		#if 1
		printf("%s : %d\r\n",__FUNCTION__,uart0Dev.len);
		#endif

		UART_Write(UART0,uart0Dev.buf,uart0Dev.len);
		
		memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));
	}
	#endif

}


void UART0_Init(void)
{
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, RX_TIMEOUT_CNT);

	/* Set UART FIFO RX interrupt trigger level to 4-bytes*/
	#if defined (RX_UNKNOWN_LENGTH)
    UART0->FIFO = ((UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);
	#else
	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;
	#endif

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);
	
	memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

	#if defined (RX_UNKNOWN_LENGTH)
	UART_WAIT_TX_EMPTY(UART0);
	#endif

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());
	
}


/*

    printf("+----------------------------------------------------------------------+\n");
    printf("|  This sample code performs how to receive unknow data length package.|\n");
    printf("|                                                                      |\n");
    printf("|   (1) Please send data to UART0 Rx(PB.12)                            |\n");
    printf("|   (2) UART will receive data by UART Rx RDA and RXTO interrupt.      |\n");
    printf("|   (3) User can modify the Rx Timeout counter RX_TIMEOUT_CNT for      |\n");
    printf("|       diffirent timeout period.                                      |\n");
    printf("|                                                                      |\n");
    printf("|   Description for RX_TIMEOUT_CNT :                                   |\n");
    printf("|   -UART data = 8 bits                                                |\n");
    printf("|   -UART Parity = None                                                |\n");
    printf("|   -RX_TIMEOUT_CNT = 60                                               |\n");
    printf("|     If there is no data comes in 60 baudrate clock,                  |\n");
    printf("|     the UART Rx timeout interrupt flag will be set to 1.             |\n");
    printf("|     RX_TIMEOUT_CNT = 60 = 6 * ( 1 start + 8 data bits + 1 stop bit ) |\n");
    printf("|                         = 6 bytes data transmittion times            |\n");
    printf("+----------------------------------------------------------------------+\n\n");

*/

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{

    SYS_Init();
	
    UART0_Init();
    UART1_Init();
	
	GPIO_Init();
	
	TIMER0_Init();
	
    /* Got no where to go, just loop forever */
    while(1)
    {

		UART0_Process();
    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
