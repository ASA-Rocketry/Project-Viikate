project('embedded_code', 'c',
  default_options: ['c_std=c11', 'warning_level=2']
)

hal_backend = get_option('hal_backend')

if hal_backend == 'mock'
  add_project_arguments('-DHAL_BACKEND_mock', language: 'c')
elif hal_backend == 'teensy41'
  add_project_arguments('-DHAL_BACKEND_teensy41', language: 'c')
else
  error('Unknown HAL backend: ' + hal_backend)
endif

subdir('shared')
subdir('hal')
subdir('firmware')
