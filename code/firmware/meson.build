firmware_inc = include_directories('include')

firmware_exe = executable(
  'firmware',
  sources: files('src/main.c'),
  include_directories: firmware_inc,
  dependencies: [
    hal_dep,
    shared_dep
  ],
)
