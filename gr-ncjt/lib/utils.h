/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <volk/volk.h>

#ifndef GR_RCWIFI_UTILS_H
#define GR_RCWIFI_UTILS_H

#define GR_FSEEK fseeko
#define GR_FTELL ftello
#define GR_FSTAT fstat
#define GR_FILENO fileno
#define GR_STAT stat

#define dout d_debug && std::cout
#define mylog(msg) do { if(d_log) { GR_LOG_INFO(d_logger, msg); }} while(0);

int *
malloc_int(int size);

float *
malloc_float(int size);

double *
malloc_double(int size);

gr_complex *
malloc_complex(int size);

// void
// free(void *b);

unsigned char
crc8(void const *mem, size_t len);


#endif //GR_RCWIFI_UTILS_H
