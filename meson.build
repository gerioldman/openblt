project(
    'openblt',
    'c',
    version: '0.0.0',
    default_options: {
        'c_std': 'gnu99',
        'optimization' : 3,
        'debug' : true
    }
)

if not meson.is_cross_build()
# Include Host related build files
subdir('Host')
endif

if meson.is_cross_build()
# Include Target related build files
subdir('Target')
endif