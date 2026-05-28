#include "teensy41.h"

#include "hal.h"

#define IOMUXC_ALT_GPIO 5
#define IOMUXC_PAD_CTL_DEFAULT 0x10B0u

#define GPIO_DR(base) (*(volatile u32 *)((base) + 0x000u))
#define GPIO_GDIR(base) (*(volatile u32 *)((base) + 0x004u))
#define GPIO_PSR(base) (*(volatile u32 *)((base) + 0x008u))

#define GPIO1_BASE 0x401B8000u
#define GPIO2_BASE 0x401BC000u
#define GPIO3_BASE 0x401C0000u
#define GPIO4_BASE 0x401C4000u

typedef struct {
    u32 gpio_base;
    u32 gpio_bit;
    u32 iomux_mux;
} PinMapping;

#define P(mux_addr, base, bit) \
    { .iomux_mux = (mux_addr), .gpio_base = (base), .gpio_bit = (bit) }

static const PinMapping pin_map[MAX_TEENSY41_PIN_COUNT] = {
    [0] = P(0x401F8020u, GPIO1_BASE, 3),  // AD_B0_03  GPIO1_3
    [1] = P(0x401F801Cu, GPIO1_BASE, 2),  // AD_B0_02  GPIO1_2
    [2] = P(0x401F8138u, GPIO4_BASE, 4),  // EMC_04    GPIO4_4
    [3] = P(0x401F813Cu, GPIO4_BASE, 5),  // EMC_05    GPIO4_5
    [4] = P(0x401F8140u, GPIO4_BASE, 6),  // EMC_06    GPIO4_6
    [5] = P(0x401F8148u, GPIO4_BASE, 8),  // EMC_08    GPIO4_8
    [6] = P(0x401F80D0u, GPIO2_BASE, 10),  // B0_10     GPIO2_10
    [7] = P(0x401F80ECu, GPIO2_BASE, 17),  // B1_01     GPIO2_17
    [8] = P(0x401F80E8u, GPIO2_BASE, 16),  // B1_00     GPIO2_16
    [9] = P(0x401F80D4u, GPIO2_BASE, 11),  // B0_11     GPIO2_11
    [10] = P(0x401F80A8u, GPIO2_BASE, 0),  // B0_00     GPIO2_0
    [11] = P(0x401F80B0u, GPIO2_BASE, 2),  // B0_02     GPIO2_2
    [12] = P(0x401F80ACu, GPIO2_BASE, 1),  // B0_01     GPIO2_1
    [13] = P(0x401F80B4u, GPIO2_BASE, 3),  // B0_03     GPIO2_3 (LED)
    [14] = P(0x401F805Cu, GPIO1_BASE, 18),  // AD_B1_02  GPIO1_18
    [15] = P(0x401F8060u, GPIO1_BASE, 19),  // AD_B1_03  GPIO1_19
    [16] = P(0x401F8070u, GPIO1_BASE, 23),  // AD_B1_07  GPIO1_23
    [17] = P(0x401F806Cu, GPIO1_BASE, 22),  // AD_B1_06  GPIO1_22
    [18] = P(0x401F8058u, GPIO1_BASE, 17),  // AD_B1_01  GPIO1_17
    [19] = P(0x401F8054u, GPIO1_BASE, 16),  // AD_B1_00  GPIO1_16
    [20] = P(0x401F807Cu, GPIO1_BASE, 26),  // AD_B1_10  GPIO1_26
    [21] = P(0x401F8080u, GPIO1_BASE, 27),  // AD_B1_11  GPIO1_27
    [22] = P(0x401F8074u, GPIO1_BASE, 24),  // AD_B1_08  GPIO1_24
    [23] = P(0x401F8078u, GPIO1_BASE, 25),  // AD_B1_09  GPIO1_25
    [24] = P(0x401F8044u, GPIO1_BASE, 12),  // AD_B0_12  GPIO1_12
    [25] = P(0x401F8048u, GPIO1_BASE, 13),  // AD_B0_13  GPIO1_13
    [26] = P(0x401F808Cu, GPIO1_BASE, 30),  // AD_B1_14  GPIO1_30
    [27] = P(0x401F8090u, GPIO1_BASE, 31),  // AD_B1_15  GPIO1_31
    [28] = P(0x401F81A8u, GPIO3_BASE, 18),  // EMC_32    GPIO3_18
    [29] = P(0x401F81A4u, GPIO4_BASE, 31),  // EMC_31    GPIO4_31
    [30] = P(0x401F81BCu, GPIO3_BASE, 23),  // EMC_37    GPIO3_23
    [31] = P(0x401F81B8u, GPIO3_BASE, 22),  // EMC_36    GPIO3_22
    [32] = P(0x401F80D8u, GPIO2_BASE, 12),  // B0_12     GPIO2_12
    [33] = P(0x401F8144u, GPIO4_BASE, 7),  // EMC_07    GPIO4_7
    [34] = P(0x401F811Cu, GPIO2_BASE, 29),  // B1_13     GPIO2_29
    [35] = P(0x401F8118u, GPIO2_BASE, 28),  // B1_12     GPIO2_28
    [36] = P(0x401F80F0u, GPIO2_BASE, 18),  // B1_02     GPIO2_18
    [37] = P(0x401F80F4u, GPIO2_BASE, 19),  // B1_03     GPIO2_19
    [38] = P(0x401F8084u, GPIO1_BASE, 28),  // AD_B1_12  GPIO1_28
    [39] = P(0x401F8088u, GPIO1_BASE, 29),  // AD_B1_13  GPIO1_29
    [40] = P(0x401F8064u, GPIO1_BASE, 20),  // AD_B1_04  GPIO1_20
    [41] = P(0x401F8068u, GPIO1_BASE, 21),  // AD_B1_05  GPIO1_21
    [42] = P(0x401F81CCu, GPIO3_BASE, 15),  // SD_B0_03  GPIO3_15
    [43] = P(0x401F81C8u, GPIO3_BASE, 14),  // SD_B0_02  GPIO3_14
    [44] = P(0x401F81C4u, GPIO3_BASE, 13),  // SD_B0_01  GPIO3_13
    [45] = P(0x401F81C0u, GPIO3_BASE, 12),  // SD_B0_00  GPIO3_12
    [46] = P(0x401F81D4u, GPIO3_BASE, 17),  // SD_B0_05  GPIO3_17
    [47] = P(0x401F81D0u, GPIO3_BASE, 16),  // SD_B0_04  GPIO3_16
    [48] = P(0x401F8188u, GPIO4_BASE, 24),  // EMC_24    GPIO4_24
    [49] = P(0x401F8194u, GPIO4_BASE, 27),  // EMC_27    GPIO4_27
    [50] = P(0x401F8198u, GPIO4_BASE, 28),  // EMC_28    GPIO4_28
    [51] = P(0x401F8180u, GPIO4_BASE, 22),  // EMC_22    GPIO4_22
    [52] = P(0x401F8190u, GPIO4_BASE, 26),  // EMC_26    GPIO4_26
    [53] = P(0x401F818Cu, GPIO4_BASE, 25),  // EMC_25    GPIO4_25
    [54] = P(0x401F819Cu, GPIO4_BASE, 29),  // EMC_29    GPIO4_29
};

#undef P

void Hal_init_begin(Hal *hal) {
    ASSERT(
        hal->canary != hal,
        "Canary failed, this HAL was already initialized"
    );

    hal->canary = 0;
    for (int i = 0; i < MAX_TEENSY41_PIN_COUNT; i++) {
        hal->pin_modes[i] = PIN_MODE_INACTIVE;
    }
}

void Hal_init_enable_gpio_output_logical(Hal *hal, const pin_id_logical pin) {
    ASSERT(
        pin < MAX_TEENSY41_PIN_COUNT,
        "Only pins 0-%d are supported, but got %d",
        MAX_TEENSY41_PIN_COUNT - 1,
        pin
    );
    ASSERT(
        hal->pin_modes[pin] == PIN_MODE_INACTIVE,
        "Pin %d mode is already set to %s, not INACTIVE",
        pin,
        pin_mode_to_str(hal->pin_modes[pin])
    );

    const PinMapping *m = &pin_map[pin];

    *(volatile u32 *)m->iomux_mux = IOMUXC_ALT_GPIO;
    *(volatile u32 *)(m->iomux_mux + 0x200u) = IOMUXC_PAD_CTL_DEFAULT;

    GPIO_GDIR(m->gpio_base) |= (1u << m->gpio_bit);

    hal->pin_modes[pin] = PIN_MODE_OUTPUT;
}

void Hal_init_end(Hal *hal) {
    ASSERT(
        hal->canary != hal,
        "Canary failed, this HAL was already initialized"
    );
    ASSERT(
        hal->canary == 0,
        "Canary failed, this HAL was not being initialized"
    );

    hal->canary = hal;
}

void Hal_set_gpio_logical(
    Hal *hal,
    const pin_id_logical pin,
    const bool state
) {
    ASSERT(
        pin < MAX_TEENSY41_PIN_COUNT,
        "Only pins 0-%d are supported, but got %d",
        MAX_TEENSY41_PIN_COUNT - 1,
        pin
    );
    ASSERT(
        hal->pin_modes[pin] == PIN_MODE_OUTPUT,
        "Pin %d must have output mode to be written, but has %s instead",
        pin,
        pin_mode_to_str(hal->pin_modes[pin])
    );

    const PinMapping *m = &pin_map[pin];
    if (state) {
        GPIO_DR(m->gpio_base) |= (1u << m->gpio_bit);
    } else {
        GPIO_DR(m->gpio_base) &= ~(1u << m->gpio_bit);
    }
}

bool Hal_get_gpio(const Hal *hal, const pin_id_logical pin) {
    (void)hal;
    ASSERT(
        pin < MAX_TEENSY41_PIN_COUNT,
        "Only pins 0-%d are supported, but got %d",
        MAX_TEENSY41_PIN_COUNT - 1,
        pin
    );

    const PinMapping *m = &pin_map[pin];
    return (GPIO_PSR(m->gpio_base) >> m->gpio_bit) & 1u;
}

void Hal_blocking_sleep_ms(Hal *hal, int ms) {
    (void)hal;
    for (int i = 0; i < ms; i++) {
        for (volatile int j = 0; j < 30000; j++) {}
    }
}
