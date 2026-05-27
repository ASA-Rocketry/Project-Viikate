project('embedded_code', 'c',
  default_options: ['c_std=c11', 'warning_level=2']
)

hal_backend = get_option('hal_backend')

subdir('shared')
subdir('hal')
subdir('firmware')
