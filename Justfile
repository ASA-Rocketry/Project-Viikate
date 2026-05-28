build_dir_host := "code/build-host"
build_dir_teensy := "code/build-teensy"
hex_file := build_dir_teensy + "/firmware/firmware.hex"
elf_file := build_dir_teensy + "/firmware/firmware"
asm_file := build_dir_teensy + "/firmware/firmware.asm"

default:
    just --list

host:
    meson setup {{build_dir_host}} code -Dhal_backend=mock && ninja -C {{build_dir_host}}

run: host
    ./{{build_dir_host}}/firmware/firmware

teensy:
    meson setup {{build_dir_teensy}} code --cross-file code/cross/teensy.ini -Dhal_backend=teensy41 && ninja -C {{build_dir_teensy}}

upload: teensy
    teensy_loader_cli --mcu=imxrt1062 -v -w {{hex_file}}

asm: teensy
    arm-none-eabi-objdump -d -S -C {{elf_file}} > {{asm_file}}

clean:
    rm -rf {{build_dir_host}} {{build_dir_teensy}}
