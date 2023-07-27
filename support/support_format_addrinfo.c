/* Convert struct addrinfo values to a string.
   Copyright (C) 2016-2017 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, see
   <http://www.gnu.org/licenses/>.  */

#include <support/format_nss.h>

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <support/support.h>
#include <support/xmemstream.h>

static size_t
socket_address_length (int family)
{
  switch (family)
    {
    case AF_INET:
      return sizeof (struct sockaddr_in);
    case AF_INET6:
      return sizeof (struct sockaddr_in6);
    default:
      return -1;
    }
}

static void
format_ai_flags (FILE *out, struct addrinfo *ai, int flag, const char *name,
                 int * flags_printed)
{
  if ((ai->ai_flags & flag) != 0)
    fprintf (out, " %s", name);
  *flags_printed |= flag;
}

static void
format_ai_one (FILE *out, struct addrinfo *ai, int *flags)
{
  /* ai_flags */
  if (ai->ai_flags != *flags)
    {
      fprintf (out, "flags:");
      int flags_printed = 0;
#define FLAG(flag) format_ai_flags (out, ai, flag, #flag, &flags_printed)
      FLAG (AI_PASSIVE);
      FLAG (AI_CANONNAME);
      FLAG (AI_NUMERICHOST);
      FLAG (AI_V4MAPPED);
      FLAG (AI_ALL);
      FLAG (AI_ADDRCONFIG);
      FLAG (AI_IDN);
      FLAG (AI_CANONIDN);
      FLAG (AI_IDN_ALLOW_UNASSIGNED);
      FLAG (AI_IDN_USE_STD3_ASCII_RULES);
      FLAG (AI_NUMERICSERV);
#undef FLAG
      int remaining = ai->ai_flags & ~flags_printed;
      if (remaining != 0)
        fprintf (out, " %08x", remaining);
      fprintf (out, "\n");
      *flags = ai->ai_flags;
    }

  {
    char type_buf[32];
    const char *type_str;
    char proto_buf[32];
    const char *proto_str;

    /* ai_socktype */
    switch (ai->ai_socktype)
      {
      case SOCK_RAW:
        type_str = "RAW";
        break;
      case SOCK_DGRAM:
        type_str = "DGRAM";
        break;
      case SOCK_STREAM:
        type_str = "STREAM";
        break;
      default:
        snprintf (type_buf, sizeof (type_buf), "%d", ai->ai_socktype);
        type_str = type_buf;
      }

    /* ai_protocol */
    switch (ai->ai_protocol)
      {
      case IPPROTO_IP:
        proto_str = "IP";
        break;
      case IPPROTO_UDP:
        proto_str = "UDP";
        break;
      case IPPROTO_TCP:
        proto_str = "TCP";
        break;
      default:
        snprintf (proto_buf, sizeof (proto_buf), "%d", ai->ai_protocol);
        proto_str = proto_buf;
      }
    fprintf (out, "address: %s/%s", type_str, proto_str);
  }

  /* ai_addrlen */
  if (ai->ai_addrlen != socket_address_length (ai->ai_family))
    {
      char *family = support_format_address_family (ai->ai_family);
      fprintf (out, "error: invalid address length %d for %s\n",
               ai->ai_addrlen, family);
      free (family);
    }

  /* ai_addr */
  {
    char buf[128];
    uint16_t port;
    const char *ret;
    switch (ai->ai_family)
      {
      case AF_INET:
        {
          struct sockaddr_in *sin = (struct sockaddr_in *) ai->ai_addr;
          ret = inet_ntop (AF_INET, &sin->sin_addr, buf, sizeof (buf));
          port = sin->sin_port;
        }
        break;
      case AF_INET6:
        {
          struct sockaddr_in6 *sin = (struct sockaddr_in6 *) ai->ai_addr;
          ret = inet_ntop (AF_INET6, &sin->sin6_addr, buf, sizeof (buf));
          port = sin->sin6_port;
        }
        break;
      default:
        errno = EAFNOSUPPORT;
        ret = NULL;
      }
    if (ret == NULL)
        fprintf (out, "error: inet_top failed: %m\n");
    else
      fprintf (out, " %s %u\n", buf, ntohs (port));
  }

  /* ai_canonname */
  if (ai->ai_canonname != NULL)
    fprintf (out, "canonname: %s\n", ai->ai_canonname);
}

/* Format all the addresses in one address family.  */
static void
format_ai_family (FILE *out, struct addrinfo *ai, int family, int *flags)
{
  while (ai)
    {
      if (ai->ai_family == family)
        format_ai_one (out, ai, flags);
      ai = ai->ai_next;
    }
}

char *
support_format_addrinfo (struct addrinfo *ai, int ret)
{
  int errno_copy = errno;

  struct xmemstream mem;
  xopen_memstream (&mem);
  if (ret != 0)
    {
      fprintf (mem.out, "error: %s\n", gai_strerror (ret));
      if (ret == EAI_SYSTEM)
        {
          errno = errno_copy;
          fprintf (mem.out, "error: %m\n");
        }
    }
  else
    {
      int flags = 0;
      format_ai_family (mem.out, ai, AF_INET, &flags);
      format_ai_family (mem.out, ai, AF_INET6, &flags);
    }

  xfclose_memstream (&mem);
  return mem.buffer;
}
