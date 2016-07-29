/****************************************************************************
 * examples/nettest/nettest-client.c
 *
 *   Copyright (C) 2007, 2011-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "config.h"

#include <sys/socket.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <arpa/inet.h>

#include "nettest.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void send_client(void)
{
#ifdef CONFIG_EXAMPLES_NETTEST_IPv6
  struct sockaddr_in6 myaddr;
#else
  struct sockaddr_in myaddr;
#endif
  char *outbuf;
#ifndef CONFIG_EXAMPLES_NETTEST_PERFORMANCE
  char *inbuf;
#endif
  int sockfd;
  socklen_t addrlen;
  int nbytessent;
#ifndef CONFIG_EXAMPLES_NETTEST_PERFORMANCE
  int nbytesrecvd;
  int totalbytesrecvd;
#endif
  int ch;
  int i;

  /* Allocate buffers */

  outbuf = (char*)malloc(SENDSIZE);
#ifndef CONFIG_EXAMPLES_NETTEST_PERFORMANCE
  inbuf  = (char*)malloc(SENDSIZE);
  if (!outbuf || !inbuf)
#else
  if (!outbuf)
#endif
    {
      printf("client: failed to allocate buffers\n");
      exit(1);
    }

  /* Create a new TCP socket */

  sockfd = socket(PF_INETX, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      printf("client socket failure %d\n", errno);
      goto errout_with_buffers;
    }

  /* Connect the socket to the server */

#ifdef CONFIG_EXAMPLES_NETTEST_IPv6
  myaddr.sin6_family            = AF_INET6;
  myaddr.sin6_port              = HTONS(PORTNO);

#ifdef CONFIG_EXAMPLES_NETTEST_LOOPBACK
  myaddr.sin6_addr.s6_addr16[0] = 0;
  myaddr.sin6_addr.s6_addr16[1] = 0;
  myaddr.sin6_addr.s6_addr16[2] = 0;
  myaddr.sin6_addr.s6_addr16[3] = 0;
  myaddr.sin6_addr.s6_addr16[4] = 0;
  myaddr.sin6_addr.s6_addr16[5] = 0;
  myaddr.sin6_addr.s6_addr16[6] = 0;
  myaddr.sin6_addr.s6_addr16[7] = HTONS(1);
#else
  myaddr.sin6_addr.s6_addr16[0] = HTONS(CONFIG_EXAMPLES_NETTEST_CLIENTIPv6ADDR_1);
  myaddr.sin6_addr.s6_addr16[1] = HTONS(CONFIG_EXAMPLES_NETTEST_CLIENTIPv6ADDR_2);
  myaddr.sin6_addr.s6_addr16[2] = HTONS(CONFIG_EXAMPLES_NETTEST_CLIENTIPv6ADDR_3);
  myaddr.sin6_addr.s6_addr16[3] = HTONS(CONFIG_EXAMPLES_NETTEST_CLIENTIPv6ADDR_4);
  myaddr.sin6_addr.s6_addr16[4] = HTONS(CONFIG_EXAMPLES_NETTEST_CLIENTIPv6ADDR_5);
  myaddr.sin6_addr.s6_addr16[5] = HTONS(CONFIG_EXAMPLES_NETTEST_CLIENTIPv6ADDR_6);
  myaddr.sin6_addr.s6_addr16[6] = HTONS(CONFIG_EXAMPLES_NETTEST_CLIENTIPv6ADDR_7);
  myaddr.sin6_addr.s6_addr16[7] = HTONS(CONFIG_EXAMPLES_NETTEST_CLIENTIPv6ADDR_8);
#endif

  addrlen = sizeof(struct sockaddr_in6);
#else
  myaddr.sin_family             = AF_INET;
  myaddr.sin_port               = HTONS(PORTNO);

#ifdef CONFIG_EXAMPLES_NETTEST_LOOPBACK
  myaddr.sin_addr.s_addr        = HTONL(0x7f000001);
#else
  myaddr.sin_addr.s_addr        = HTONL(CONFIG_EXAMPLES_NETTEST_CLIENTIP);
#endif

  addrlen = sizeof(struct sockaddr_in);
#endif

  printf("client: Connecting...\n");
  if (connect( sockfd, (struct sockaddr*)&myaddr, addrlen) < 0)
    {
      printf("client: connect failure: %d\n", errno);
      goto errout_with_socket;
    }

  printf("client: Connected\n");

  /* Initialize the buffer */

  ch = 0x20;
  for (i = 0; i < SENDSIZE; i++ )
    {
      outbuf[i] = ch;
      if (++ch > 0x7e)
        {
          ch = 0x20;
        }
    }

#ifdef CONFIG_EXAMPLES_NETTEST_PERFORMANCE
  /* Then send messages forever */

  for (;;)
    {
      nbytessent = send(sockfd, outbuf, SENDSIZE, 0);
      if (nbytessent < 0)
        {
          printf("client: send failed: %d\n", errno);
          goto errout_with_socket;
        }
      else if (nbytessent != SENDSIZE)
        {
          printf("client: Bad send length=%d: %d of \n",
                  nbytessent, SENDSIZE);
          goto errout_with_socket;
        }

      printf("Sent %d bytes\n", nbytessent);
    }
#else
  /* Then send and receive one message */

  printf("client: Sending %d bytes\n", SENDSIZE);
  nbytessent = send(sockfd, outbuf, SENDSIZE, 0);
  printf("client: Sent %d bytes\n", nbytessent);

  if (nbytessent < 0)
    {
      printf("client: send failed: %d\n", errno);
      goto errout_with_socket;
    }
  else if (nbytessent != SENDSIZE)
    {
      printf("client: Bad send length: %d Expected: %d\n", nbytessent, SENDSIZE);
      goto errout_with_socket;
    }

  totalbytesrecvd = 0;
  do
    {
      printf("client: Receiving...\n");
      nbytesrecvd = recv(sockfd, &inbuf[totalbytesrecvd], SENDSIZE - totalbytesrecvd, 0);

      if (nbytesrecvd < 0)
        {
          printf("client: recv failed: %d\n", errno);
          goto errout_with_socket;
        }
      else if (nbytesrecvd == 0)
        {
          printf("client: The server closed the connection\n");
          goto errout_with_socket;
        }

      totalbytesrecvd += nbytesrecvd;
      printf("client: Received %d of %d bytes\n", totalbytesrecvd, SENDSIZE);
    }
  while (totalbytesrecvd < SENDSIZE);

  if (totalbytesrecvd != SENDSIZE)
    {
      printf("client: Bad recv length: %d Expected: %d\n", totalbytesrecvd, SENDSIZE);
      goto errout_with_socket;
    }
  else if (memcmp(inbuf, outbuf, SENDSIZE) != 0)
    {
      printf("client: Received buffer does not match sent buffer\n");
      goto errout_with_socket;
    }

  close(sockfd);
  free(outbuf);
#ifndef CONFIG_EXAMPLES_NETTEST_PERFORMANCE
  free(inbuf);
#endif
  return;
#endif

errout_with_socket:
  close(sockfd);

errout_with_buffers:
  free(outbuf);
#ifndef CONFIG_EXAMPLES_NETTEST_PERFORMANCE
  free(inbuf);
#endif
  exit(1);
}
