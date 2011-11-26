#ident "$Id$"
/* ----------------------------------------------------------------------- *
 *   
 *   Copyright 2005 H. Peter Anvin - All Rights Reserved
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 53 Temple Place Ste 330,
 *   Boston MA 02111-1307, USA; either version 2 of the License, or
 *   (at your option) any later version; incorporated herein by reference.
 *
 * ----------------------------------------------------------------------- */

/*
 * fatattr.c
 *
 * Display or change attributes on a FAT filesystem, similar to the
 * DOS attrib command.
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#include <sysexits.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/msdos_fs.h>

#ifndef FAT_IOCTL_GET_ATTRIBUTES
# define FAT_IOCTL_GET_ATTRIBUTES        _IOR('r', 0x10, __u32)
#endif
#ifndef FAT_IOCTL_SET_ATTRIBUTES
# define FAT_IOCTL_SET_ATTRIBUTES        _IOW('r', 0x11, __u32)
#endif

const char *program;
/* Currently supports only the FAT flags, not the NTFS ones */
const char bit_to_char[] = "rhsvda67";

static int
chattr(const char *file, uint32_t s_attr, uint32_t c_attr)
{
  int fd = -1;
  uint32_t attr, nattr;
  int e;

  fd = open(file, O_RDONLY);
  if ( fd < 0 )
    goto err;

  if ( ioctl(fd, FAT_IOCTL_GET_ATTRIBUTES, &attr) )
    goto err;

  nattr = (attr & ~c_attr) | s_attr;

  if ( nattr != attr ) {
    if ( ioctl(fd, FAT_IOCTL_SET_ATTRIBUTES, &nattr) )
      goto err;
  }

  close(fd);
  return 0;

 err:
  e = errno;
  if ( fd >= 0 )
    close(fd);

  errno = e;
  return -1;
}

static int
lsattr(const char *file)
{
  int fd = -1;
  uint32_t attr;
  int i, e;

  fd = open(file, O_RDONLY);
  if ( fd < 0 )
    goto err;

  if ( ioctl(fd, FAT_IOCTL_GET_ATTRIBUTES, &attr) )
    goto err;

  close(fd);

  for ( i = 0 ; bit_to_char[i] ; i++ ) {
    putchar( (attr & 1) ? bit_to_char[i] : ' ' );
    attr >>= 1;
  }

  putchar(' ');
  puts(file);
  return 0;

 err:
  e = errno;
  if ( fd >= 0 )
    close(fd);
  errno = e;
  return -1;
}

static uint32_t
parse_attr(const char *attrs)
{
  uint32_t attr = 0;
  const char *p;

  while ( *attrs ) {
    p = strchr(bit_to_char, *attrs);
    if ( !p ) {
      fprintf(stderr, "%s: unknown attribute: '%c'\n", program, *attrs);
      exit(EX_USAGE);
    }

    attr |= (uint32_t)1 << (p-bit_to_char);
    attrs++;
  }

  return attr;
}

static void __attribute__((noreturn)) usage(void)
{
  fprintf(stderr, "Usage: %s [-+%s] files...\n", program, bit_to_char);
  exit(EX_USAGE);
}

int main(int argc, char *argv[])
{
  uint32_t s_attr = 0;		/* Attributes to set */
  uint32_t c_attr = 0;		/* Attributes to clear */
  int i;
  int files = 0;		/* Files processed */
  int e;

  program = argv[0];

  for ( i = 1 ; i < argc ; i++ ) {
    if ( argv[i][0] == '-' )
      c_attr |= parse_attr(argv[i]+1);
    else if ( argv[i][0] == '+' )
      s_attr |= parse_attr(argv[i]+1);
    else {
      e = 0;

      if ( c_attr | s_attr )
	e = chattr(argv[i], s_attr, c_attr);
      else
	e = lsattr(argv[i]);

      if ( e ) {
	perror(argv[i]);
	return EX_DATAERR;
      }

      files++;
    }
  }

  if ( !files )
    usage();

  return 0;
}

