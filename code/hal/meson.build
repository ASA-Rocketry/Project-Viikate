hal_inc = include_directories('include')

hal_sources = files('src/hal.c')

if hal_backend == 'mock'
  hal_sources += files('src/mock.c')
elif hal_backend == 'teensy41'
  hal_sources += files('src/teensy41.c')
else
  error('Unknown HAL backend: ' + hal_backend)
endif

hal_lib = static_library(
  'hal',
  sources: hal_sources,
  include_directories: hal_inc,
  dependencies: shared_dep,
)

hal_dep = declare_dependency(
  include_directories: hal_inc,
  link_with: hal_lib,
  dependencies: shared_dep,
)
