#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#pragma once

typedef unsigned char bool;

typedef int i32;
typedef short i16;
typedef char i8;

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

typedef float f32;
typedef double f64;

static_assert(sizeof(bool) == 1, "bool must be 1 byte");

static_assert(sizeof(i32) == 4, "i32 must be 4 bytes");
static_assert(sizeof(i16) == 2, "i16 must be 2 bytes");
static_assert(sizeof(i8) == 1, "i8 must be 1 byte");

static_assert(sizeof(u32) == 4, "u32 must be 4 bytes");
static_assert(sizeof(u16) == 2, "u16 must be 2 bytes");
static_assert(sizeof(u8) == 1, "u8 must be 1 byte");

static_assert(sizeof(f32) == 4, "f32 must be 4 bytes");
static_assert(sizeof(f64) == 8, "f64 must be 8 bytes");

#define true ((bool)1)
#define false ((bool)0)

/// XXX(Artur): these currently depend on stdio, this is problematic in the
/// actual Teensy, so open to ideas on how this can be separated.

#define ASSERT(expr, msg, ...) \
    do { \
        if (!(expr)) { \
            fprintf( \
                stderr, \
                "[ASSERT FAILED] %s:%d in %s()\n", \
                __FILE__, \
                __LINE__, \
                __func__ \
            ); \
            fprintf(stderr, msg, ##__VA_ARGS__); \
            fprintf(stderr, "\n"); \
            abort(); \
        } \
    } while (0)

#define PANIC(msg, ...) \
    do { \
        fprintf( \
            stderr, \
            "[PANIC] %s:%d in %s()\n", \
            __FILE__, \
            __LINE__, \
            __func__ \
        ); \
        fprintf(stderr, msg, ##__VA_ARGS__); \
        fprintf(stderr, "\n"); \
        abort(); \
    } while (0)

#define UNREACHABLE \
    do { \
        fprintf( \
            stderr, \
            "[UNREACHABLE] %s:%d in %s()\n", \
            __FILE__, \
            __LINE__, \
            __func__ \
        ); \
        abort(); \
        __builtin_unreachable(); \
    } while (0)
