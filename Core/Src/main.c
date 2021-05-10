/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma2d.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "socket.h"
#include "dhcp.h"
//#include "dns.h"
#include "w5500.h"
#include "mb.h"
#include "mbproto.h"
#include "mbutils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2
#define REG_HOLDING_START 1000
#define REG_HOLDING_NREGS 4
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4
#define REG_COILS_START     1000
#define REG_COILS_SIZE      16
#define REG_DISC_START     1000
#define REG_DISC_SIZE      16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t dhcp_buffer[1024];
uint8_t dns_buffer[1024];
volatile bool ip_assigned = false;
static unsigned short usRegHoldingStart = REG_HOLDING_START;
static unsigned short usRegHoldingBuf[REG_HOLDING_NREGS]={0xaa,0x55,0x5599,0x5588};
static unsigned short usRegInputStart = REG_INPUT_START;
static unsigned short usRegInputBuf[REG_INPUT_NREGS]={0,0,0x5599,0x5588};
static unsigned char ucRegCoilsBuf[REG_COILS_SIZE / 8]={0xaa,0xfe};
static unsigned char ucRegDiscBuf[REG_DISC_SIZE / 8] = { 0x98, 0x6e };
// uint8_t reg=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void W5500_Select(void) {
    HAL_GPIO_WritePin(SCS_GPIO_Port, SCS_Pin, GPIO_PIN_RESET);
}
void W5500_Unselect(void) {
    HAL_GPIO_WritePin(SCS_GPIO_Port, SCS_Pin, GPIO_PIN_SET);
}
void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi5, buff, len, HAL_MAX_DELAY);
}
void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi5, buff, len, HAL_MAX_DELAY);
}
uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}
void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}
void Callback_IPAssigned(void) {
    printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
    ip_assigned = true;
}
void Callback_IPConflict(void) {
    printf("Callback: IP conflict!\r\n");
}

void init_w5500() {
  printf("\r\ninit() called!\r\n");

  printf("Registering W5500 callbacks...\r\n");
  reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
  reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
  reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

  printf("Calling wizchip_init()...\r\n");
  uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
  wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

  printf("Calling DHCP_init()...\r\n");
  wiz_NetInfo net_info = {
    .mac  = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },
/*    .ip = {192, 168, 31, 144},
    .sn = {255, 255, 255, 0},
    .gw = {192, 168, 31, 1},*/
    .dhcp = NETINFO_DHCP
  };

  setSHAR(net_info.mac);

  DHCP_init(DHCP_SOCKET, dhcp_buffer);

  printf("Registering DHCP callbacks...\r\n");
  reg_dhcp_cbfunc(
    Callback_IPAssigned,
    Callback_IPAssigned,
    Callback_IPConflict
  );

  printf("Calling DHCP_run()...\r\n");
  uint32_t ctr = 1000000;
  while((!ip_assigned) && (ctr > 0)) {
    DHCP_run();
    ctr--;
  }
  if(!ip_assigned) {
    printf("\r\nIP was not assigned :(\r\n");
    return;
  }

  getIPfromDHCP(net_info.ip);
  getGWfromDHCP(net_info.gw);
  getSNfromDHCP(net_info.sn);

/*  uint8_t dns[4];
  getDNSfromDHCP(dns);*/

  printf("IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nDNS: %d.%d.%d.%d\r\n",
    net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
    net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
    net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3]
  );

  printf("Calling wizchip_setnetinfo()...\r\n");
  wizchip_setnetinfo(&net_info);
  printf("State Sn_CR reg: %d\r\n", WIZCHIP_READ(Sn_SR(0)));


/*  printf("Calling DNS_init()...\r\n");
  DNS_init(DNS_SOCKET, dns_buffer);*/

/*  uint8_t addr[4];
  {
    char domain_name[] = "eax.me";
    printf("Resolving domain name \"%s\"...\r\n", domain_name);
    int8_t res = DNS_run(dns, (uint8_t*)&domain_name, addr);
    if(res != 1) {
      printf("DNS_run() failed, res = %d", res);
      return;
    }
    printf("Result: %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
  }*/

 /* printf("Creating socket...\r\n");
  uint8_t http_socket = HTTP_SOCKET;
  uint8_t code = socket(http_socket, Sn_MR_TCP, 10888, 0);
  if(code != http_socket) {
    printf("socket() failed, code = %d\r\n", code);
    return;
  }

  printf("Socket created, connecting...\r\n");
  code = connect(http_socket, addr, 80);
  if(code != SOCK_OK) {
    printf("connect() failed, code = %d\r\n", code);
    close(http_socket);
    return;
  }

  printf("Connected, sending HTTP request...\r\n");
  {
    char req[] = "GET / HTTP/1.0\r\nHost: eax.me\r\n\r\n";
    uint16_t len = sizeof(req) - 1;
    uint8_t* buff = (uint8_t*)&req;
    while (len > 0) {
      printf("Sending %d bytes...\r\n", len);
      int32_t nbytes = send(http_socket, buff, len);
      if (nbytes <= 0) {
        printf("send() failed, %d returned\r\n", nbytes);
        close(http_socket);
        return;
      }
      printf("%d bytes sent!\r\n", nbytes);
      len -= nbytes;
    }
  }

  printf("Request sent. Reading response...\r\n");
  {
    char buff[32];
    for (;;)
    {
      int32_t nbytes = recv(http_socket, (uint8_t *)&buff, sizeof(buff) - 1);
      if (nbytes == SOCKERR_SOCKSTATUS) {
        printf("\r\nConnection closed.\r\n");
        break;
      }
      if (nbytes <= 0) {
        printf("\r\nrecv() failed, %d returned\r\n", nbytes);
        break;
      }
      buff[nbytes] = '\0';
      printf("%s", buff);
    }
  }
  printf("Closing socket.\r\n");
  close(http_socket);*/
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  unsigned char i;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  init_w5500();
  HAL_Delay(200);
  eMBTCPInit(502);
  HAL_Delay(200);
  eMBEnable();
  printf("\r\nModbus-TCP Start!\r\n");
    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    i=WIZCHIP_READ(Sn_SR(0));
    if (i==0) {
        do {
            HAL_Delay(100);
        } while (listen(0)==FALSE);
    } else if ((i==SOCK_LISTEN)||(i==SOCK_ESTABLISHED)) {
//        LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
        eMBPoll();
//        LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
  LL_PWR_DisableOverDriveMode();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 72, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
            case MB_REG_READ:
                while( usNRegs > 0 )
                {
                    *pucRegBuffer++ =
                            ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                    *pucRegBuffer++ =
                            ( unsigned char )( usRegHoldingBuf[iRegIndex] &
                                               0xFF );
                    iRegIndex++;
                    usNRegs--;
                }
                break;

                /* Update current register values with new values from the
                 * protocol stack. */
            case MB_REG_WRITE:
                while( usNRegs > 0 )
                {
                    usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                    usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                    iRegIndex++;
                    usNRegs--;
                }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                    ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                    ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iNCoils = ( int )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
            /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ =
                            xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
                                            ( unsigned char )( iNCoils >
                                                               8 ? 8 :
                                                               iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
                while( iNCoils > 0 )
                {
                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
                                    ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ),
                                    *pucRegBuffer++ );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_DISC_START ) &&
        ( usAddress + usNDiscrete <= REG_DISC_START + REG_DISC_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_DISC_START );
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =
                    xMBUtilGetBits( ucRegDiscBuf, usBitOffset,
                                    ( unsigned char )( iNDiscrete >
                                                       8 ? 8 : iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


int __io_putchar(int ch) {
  ITM_SendChar(ch);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
