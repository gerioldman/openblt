platform_c_args = []
platform_deps = []

if build_machine.system() == 'windows'
    platform_c_args += '-DPLATFORM_WINDOWS'
    platform_c_args += '-D_CRT_SECURE_NO_WARNINGS'
else
    platform_c_args += '-DPLATFORM_LINUX'
    platform_deps += dependency('threads')
endif

if meson.get_compiler('c').sizeof('void*') == 4
    platform_c_args += '-DPLATFORM_32BIT'
else
    platform_c_args += '-DPLATFORM_64BIT'
endif

both_libraries('seednkey', 'seednkey.c',c_args: platform_c_args, dependencies: platform_deps)