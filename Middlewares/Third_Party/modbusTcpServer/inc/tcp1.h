//! @addtogroup TCPServerSocket
//! @{
//
//****************************************************************************
//! @file tcp.h
//! @brief This contains the prototypes, macros, constants or global variables
//!        for the TCP Server Socket
//! @author Savindra Kumar(savindran1989@gmail.com)
//! @bug No known bugs.
//
//****************************************************************************
#ifndef TCP_H
#define TCP_H

//****************************************************************************
//                           Includes
//****************************************************************************
#include "lwip.h"


#include "lwip/apps/httpd.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "api.h"
#include "string.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/udp.h"
#include "lwip/netbuf.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "err.h"
#include "lwip/ip.h"
#include "lwip/ip4.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
//****************************************************************************
//                           Constants and typedefs
//****************************************************************************


//****************************************************************************
//                           Global variables
//****************************************************************************

//****************************************************************************
//                           Global Functions
//****************************************************************************
//
//! @brief Initialize TCP Server Socket and Call Modbus Application
//! @param[in]  None
//! @param[out] None
//! @return     None
//
void tcp_Init (void);

#endif // TCP_H
//****************************************************************************
//                             End of file
//****************************************************************************
//! @}
