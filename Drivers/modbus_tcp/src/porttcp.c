/*
* FreeModbus Libary: lwIP Port
* Copyright (C) 2006 Christian Walter <wolti@sil.at>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* File: $Id: porttcp.c,v 1.1 2006/08/30 23:18:07 wolti Exp $
*/

/* ----------------------- System includes ----------------------------------*/
#include <stdio.h>
#include <string.h>
#include "port.h"
#include "w5500.h"
#include "socket.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- MBAP Header --------------------------------------*/
#define MB_TCP_UID          6
#define MB_TCP_LEN          4
#define MB_TCP_FUNC         7
#define HTTP_SOCKET         0

#define MB_TCP_DEFAULT_PORT  502        
#define MB_TCP_BUF_SIZE     (256 + 7)   

/* ----------------------- Prototypes ---------------------------------------*/

uint8_t ucTCPRequestFrame [MB_TCP_BUF_SIZE]; // Регистр приема
uint16_t ucTCPRequestLen;
uint8_t ucTCPResponseFrame [MB_TCP_BUF_SIZE]; // Отправляем регистр
uint16_t ucTCPResponseLen;
uint8_t bFrameSent = FALSE;

BOOL 
xMBTCPPortInit(USHORT usTCPPort)
{
    BOOL bOkay = FALSE;

    USHORT usPort;
    if (usTCPPort == 0)
    {
        usPort = MB_TCP_DEFAULT_PORT;
    }
    else
    {
        usPort = (USHORT)usTCPPort;
    }
    printf("Creating socket...\r\n");
    uint8_t http_socket = HTTP_SOCKET;
    uint8_t code = socket(http_socket, Sn_MR_TCP, usPort, 0);
    if (code != http_socket)
    {
        printf("socket() failed, code = %d\r\n", code);
        return;
    }
    listen(HTTP_SOCKET);
    bOkay = TRUE;
    return bOkay;
}

void vMBTCPPortClose()
{
}

void vMBTCPPortDisable(void)
{
}

BOOL 
xMBTCPPortGetRequest(UCHAR **ppucMBTCPFrame, USHORT *usTCPLength)
{
    *ppucMBTCPFrame = (uint8_t *) &ucTCPRequestFrame[0];
    *usTCPLength = ucTCPRequestLen;
    /* Reset the buffer. */
    ucTCPRequestLen = 0;
    return TRUE;
}

BOOL 
xMBTCPPortSendResponse(const UCHAR *pucMBTCPFrame, USHORT usTCPLength)
{
    memcpy(ucTCPResponseFrame, pucMBTCPFrame, usTCPLength);
    ucTCPResponseLen = usTCPLength;
    bFrameSent = TRUE; 
    return bFrameSent;
    return TRUE;
}

void modbus_tcps(uint8_t sn, uint16_t port) {
  switch (getSn_SR(sn)) 
  {
  case SOCK_CLOSED:                    
    socket(sn, Sn_MR_TCP, port, 0x00); 
    break;
  case SOCK_INIT:        
    listen(sn);         
  case SOCK_ESTABLISHED: 
    if (getSn_IR(sn) & Sn_IR_CON)
    {
      setSn_IR(sn, Sn_IR_CON);
    }
    ucTCPRequestLen = getSn_RX_RSR(sn); 
    if (ucTCPRequestLen > 0)
    {
      recv(sn, ucTCPRequestFrame, ucTCPRequestLen); 
      xMBPortEventPost(EV_FRAME_RECEIVED);          
      eMBPoll();                                    
      eMBPoll();                                    
      if (bFrameSent)
      {
        bFrameSent = FALSE;
        send(sn, ucTCPResponseFrame, ucTCPResponseLen);
      }
    }
    break;
  case SOCK_CLOSE_WAIT: 
    disconnect(sn);     
    break;
  default:
    break;
  }
}