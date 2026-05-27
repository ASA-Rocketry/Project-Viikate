shared_inc = include_directories('include')

shared_lib = static_library(
  'shared',
  include_directories: shared_inc
)

shared_dep = declare_dependency(
  include_directories: shared_inc,
  link_with: shared_lib
)
