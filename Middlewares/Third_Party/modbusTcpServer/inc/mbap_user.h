//! @addtogroup ModbusTCPUserFunction
//! @{
//
//****************************************************************************
//! @file mbap_user.h
//! @brief This contains the prototypes, macros, constants or global variables
//!        for the user configuration of mbap
//! @author Savindra Kumar(savindran1989@gmail.com)
//! @bug No known bugs.
//
//****************************************************************************
#ifndef MBAP_USER_H
#define MBAP_USER_H

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
#define INPUT_REGISTER_START_ADDRESS      (0u)
#define MAX_INPUT_REGISTERS               (50u)
#define HOLDING_REGISTER_START_ADDRESS    (0u)
#define MAX_HOLDING_REGISTERS             (200)
#define DISCRETE_INPUTS_START_ADDRESS     (0u)
#define MAX_DISCRETE_INPUTS               (100u)
#define COILS_START_ADDRESS               (0u)
#define MAX_COILS                         (100u)
#define DISCRETE_INPUT_BUF_SIZE           (MAX_DISCRETE_INPUTS / 8u + 1u)
#define COILS_BUF_SIZE                    (MAX_COILS / 8u + 1u)

//****************************************************************************
//                           Global variables
//****************************************************************************
extern uint16_t g_sInputRegsBuf[MAX_INPUT_REGISTERS];
extern uint16_t g_sHoldingRegsBuf[MAX_HOLDING_REGISTERS];
extern uint8_t g_ucDiscreteInputsBuf[DISCRETE_INPUT_BUF_SIZE];
extern uint8_t g_ucCoilsBuf[COILS_BUF_SIZE];

//****************************************************************************
//                           Global Functions
//****************************************************************************
//
//! @brief Initialize modbus data
//! @param[in]   None
//! @param[out]  None
//! @return      None
//
void mu_Init(void);

#endif // MBAP_USER_H
//****************************************************************************
//                             End of file
//****************************************************************************
//! @}
