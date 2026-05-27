#include <stdio.h>
#include <stdlib.h>

#pragma once

typedef unsigned char bool;
typedef int i32;
typedef unsigned int u32;
typedef float f32;
typedef double f64;

#define true ((bool)1)
#define false ((bool)0)

/// XXX(Artur): these currently depend on stdio, this is problematic in the
/// actual Teensy, so open to ideas on how this can be separated.

#define ASSERT(expr, msg, ...)                                               \
  do {                                                                       \
    if (!(expr)) {                                                           \
      fprintf(stderr, "[ASSERT FAILED] %s:%d in %s()\n", __FILE__, __LINE__, \
              __func__);                                                     \
      fprintf(stderr, msg, ##__VA_ARGS__);                                   \
      fprintf(stderr, "\n");                                                 \
      abort();                                                               \
    }                                                                        \
  } while (0)

#define PANIC(msg, ...)                                                       \
  do {                                                                        \
    fprintf(stderr, "[PANIC] %s:%d in %s()\n", __FILE__, __LINE__, __func__); \
    fprintf(stderr, msg, ##__VA_ARGS__);                                      \
    fprintf(stderr, "\n");                                                    \
    abort();                                                                  \
  } while (0)

#define UNREACHABLE                                                      \
  do {                                                                   \
    fprintf(stderr, "[UNREACHABLE] %s:%d in %s()\n", __FILE__, __LINE__, \
            __func__);                                                   \
    abort();                                                             \
    __builtin_unreachable();                                             \
  } while (0)
