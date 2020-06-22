# M480_BSP_CMSIS_UART0_UART1
 M480_BSP_CMSIS_UART0_UART1

update @ 2020/06/22

1. modify both UART port , to receive unknown data length , under IRQ

2. test log message as below , 

![image](https://github.com/released/M480BSP_UART0_UART1/blob/master/LOG.jpg)

3. Tera term setting , 

![image](https://github.com/released/M480BSP_UART0_UART1/blob/master/tera_term_settingjpg)

==========================================================

update @ 2019/12/11

UART0 : PB12 , PB13 , UART1 : PB2 , PB3 sample code

- UART0 TX : period output log by printf

- UART0 RX : interrupt , receive unknown length ( check #define RX_UNKNOWN_LENGTH )

- UART1 TX : period output log by UART_Write and send_UART1String

- UART1 RX : interrupt , single byte