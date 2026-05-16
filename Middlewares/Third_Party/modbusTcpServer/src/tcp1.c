//! @addtogroup TCPServerSocket
//! @brief Intialise TCP Server Socket
//! @{
//!
//****************************************************************************/
//! @file tcp.c
//! @brief TCP Server Socket
//! @author Savindra Kumar(savindran1989@gmail.com)
//! @bug No known bugs.
//!
//****************************************************************************/
//****************************************************************************/
//                           Includes
//****************************************************************************/
//standard header files
#include "tcp.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <sys/socket.h>
//#include <netdb.h>
//#include <unistd.h>
//user defined header files
#include "mbap_conf.h"
#include "tcp1.h"
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

//****************************************************************************/
//                           Defines and typedefs
//****************************************************************************/
#define BUFF_SIZE_IN_BYTES   256
#define PORT_NUMBER          1502

//****************************************************************************/
//                           external variables
//****************************************************************************/


//****************************************************************************/
//                           Local variables
//****************************************************************************/


//****************************************************************************/
//                           Local Functions
//****************************************************************************/

//****************************************************************************/
//                    G L O B A L  F U N C T I O N S
//****************************************************************************/
void tcp_Init(void)
{
  uint8_t pucQuery[BUFF_SIZE_IN_BYTES];
  uint8_t pucResponse[BUFF_SIZE_IN_BYTES];
  int16_t sReturn;
  socklen_t len;
  int sock_desc;
  int temp_sock_desc;
  struct sockaddr_in server;
  struct sockaddr_in client;
  
  memset(&server, 0, sizeof(server));
  memset(&client, 0, sizeof(client));
  
  sock_desc = socket(AF_INET, SOCK_STREAM, 0);
  
  if (sock_desc == -1)
  {
    // print  ("Error in socket creation");
    //  osDelay(1);
    
    return;
  }
  
  server.sin_family      = AF_INET;
  server.sin_addr.s_addr = htonl(INADDR_ANY);
  server.sin_port        = htons(PORT_NUMBER);
  
  sReturn = bind(sock_desc, (struct sockaddr*)&server, sizeof(server));
  
  if (-1 == sReturn)
  {
    
    //  printf("Error in binding");
    return;
  }
  
  sReturn = listen(sock_desc, 20);
  
  if (-1 == sReturn)
  {
    
    //   printf("Error in listening");
    return;
  }
  
  len = sizeof(client);
  
CLIENT_REQUEST:
  while ( (temp_sock_desc = accept(sock_desc, (struct sockaddr*)&client, &len)) )
  {
    //  printf("\nClient connected\n");
    ////added for led indicator
    HAL_GPIO_WritePin(LED_ACK_GPIO_Port,LED_ACK_Pin,0);
    ////added for led indicator
    while (1)
    {
      uint16_t usResponseLength = 0;
      
      sReturn = recv(temp_sock_desc, pucQuery, BUFF_SIZE_IN_BYTES, 0);
      
      if (0 == sReturn)
      {
        ////added for led indicator
        HAL_GPIO_WritePin(LED_ACK_GPIO_Port,LED_ACK_Pin,1);
        ////added for led indicator
        //   printf("\nConnection closed\n");
        close(temp_sock_desc);
        break;
      }
      else if (sReturn < 0)
      {
        ////added for led indicator
        HAL_GPIO_WritePin(LED_ACK_GPIO_Port,LED_ACK_Pin,1);
        ////added for led indicator
        //  printf("\nConnection reset\n");
        close(temp_sock_desc);
        break;
      }
      else
      {
        //read successfully
      }
      
      usResponseLength = mbap_ProcessRequest(pucQuery, sReturn, pucResponse);
      
      if (0 != usResponseLength)
      {
        sReturn = send(temp_sock_desc, pucResponse, usResponseLength, 0);
        
        if (sReturn < 0)
        {
          //   printf("\nsend failed\n");
        }
      }//end if
    }//end while
  }//end while
  
  
  if (temp_sock_desc < 0)
  {
    //  printf("accpet failed");
    goto CLIENT_REQUEST;
  }
  
  
  exit(0);
}//end TcpInit

//****************************************************************************/
//                           L O C A L  F U N C T I O N S
//****************************************************************************/

//****************************************************************************/
//                             End of file
//****************************************************************************/
/** @}*/
