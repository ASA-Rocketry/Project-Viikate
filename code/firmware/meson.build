firmware_sources = files('src/main.c')
firmware_link_args = []

if hal_backend == 'teensy41'
  firmware_sources += files('src/startup.c')

  if meson.is_cross_build()
    firmware_link_args += [
      '-Wl,-T,' + meson.current_source_dir() + '/imxrt1062.ld',
    ]
  endif
endif

firmware_exe = executable(
  'firmware',
  sources: firmware_sources,
  dependencies: [
    hal_dep,
    shared_dep,
  ],
  link_args: firmware_link_args,
)

if meson.is_cross_build() and hal_backend == 'teensy41'
  objcopy = find_program('arm-none-eabi-objcopy')

  custom_target(
    'firmware_hex',
    input: firmware_exe,
    output: 'firmware.hex',
    command: [objcopy, '-O', 'ihex', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
  )

  custom_target(
    'firmware_bin',
    input: firmware_exe,
    output: 'firmware.bin',
    command: [objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
  )
endif
