#pragma once

#include <defs.h>

#define DEFAULT_VECTOR_SIZE 64

#define VECTOR_DECLARE(T, NAME, CAPACITY) \
    static_assert((CAPACITY) > 0, "capacity of a vector must be > 0"); \
    typedef struct { \
        T data[CAPACITY]; \
        u32 length; \
    } NAME; \
\
    __attribute__((warn_unused_result)) static inline NAME NAME##_init() { \
        NAME v = {0}; \
        return v; \
    } \
\
    __attribute__((warn_unused_result)) static inline bool NAME##_push( \
        NAME *v, \
        T value \
    ) { \
        if (v->length >= CAPACITY) \
            return false; \
        v->data[v->length++] = value; \
        return true; \
    } \
\
    __attribute__((warn_unused_result)) static inline bool NAME##_pop( \
        NAME *v, \
        T *into \
    ) { \
        if (v->length <= 0) \
            return false; \
        *into = v->data[--v->length]; \
        return true; \
    }

VECTOR_DECLARE(bool, Vector_bool, DEFAULT_VECTOR_SIZE);

VECTOR_DECLARE(i8, Vector_i8, DEFAULT_VECTOR_SIZE);
VECTOR_DECLARE(i16, Vector_i16, DEFAULT_VECTOR_SIZE);
VECTOR_DECLARE(i32, Vector_i32, DEFAULT_VECTOR_SIZE);

VECTOR_DECLARE(u8, Vector_u8, DEFAULT_VECTOR_SIZE);
VECTOR_DECLARE(u16, Vector_u16, DEFAULT_VECTOR_SIZE);
VECTOR_DECLARE(u32, Vector_u32, DEFAULT_VECTOR_SIZE);
