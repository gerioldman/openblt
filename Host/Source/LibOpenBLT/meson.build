platform_c_args = []
platform_includes = []
platform_sources = []
platform_deps = []

if build_machine.system() == 'windows'
    platform_c_args += '-DPLATFORM_WINDOWS'
    platform_c_args += '-D_CRT_SECURE_NO_WARNINGS'
    platform_includes += include_directories(
        [
            'port/windows',
            'port/windows/canif',
            'port/windows/canif/ixxat',
            'port/windows/canif/kvaser',
            'port/windows/canif/lawicel',
            'port/windows/canif/peak',
            'port/windows/canif/vector',
        ]
    )
    platform_sources += files(
        [
            'port/windows/critutil.c',
            'port/windows/netaccess.c',
            'port/windows/serialport.c',
            'port/windows/timeutil.c',
            'port/windows/usbbulk.c',
            'port/windows/xcpprotect.c',
            'port/windows/canif/ixxat/vcidriver.c',
            'port/windows/canif/kvaser/leaflight.c',
            'port/windows/canif/lawicel/canusb.c',
            'port/windows/canif/peak/pcanusb.c',
            'port/windows/canif/vector/xldriver.c',
            'port/windows/canif/vector/xlLoadlib.c'
        ]
    )
    platform_deps += meson.get_compiler('c').find_library('ws2_32')
    platform_deps += meson.get_compiler('c').find_library('winusb')
    platform_deps += meson.get_compiler('c').find_library('setupapi')
else
    platform_c_args += '-DPLATFORM_LINUX'
    platform_includes += include_directories(
        [
            'port/linux',
            'port/linux/canif',
            'port/linux/canif/socketcan'
        ]
    )
    platform_sources += files(
        [
            'port/linux/critutil.c',
            'port/linux/netaccess.c',
            'port/linux/serialport.c',
            'port/linux/timeutil.c',
            'port/linux/usbbulk.c',
            'port/linux/xcpprotect.c',
            'port/linux/canif/socketcan/socketcan.c'
        ]
    )
    platform_deps += dependency('threads')
    platform_deps += dependency('libusb-1.0')
endif

if meson.get_compiler('c').sizeof('void*') == 4
    platform_c_args += '-DPLATFORM_32BIT'
else
    platform_c_args += '-DPLATFORM_64BIT'
endif

libopenblt = both_libraries(
    'openblt',
    include_directories: [
        platform_includes,
        '.'
    ],
    sources: [
        platform_sources,
        'aes256.c',
        'candriver.c',
        'firmware.c',
        'openblt.c',
        'session.c',
        'srecparser.c',
        'util.c',
        'xcploader.c',
        'xcptpcan.c',
        'xcptpmbrtu.c',
        'xcptpnet.c',
        'xcptpuart.c',
        'xcptpusb.c'
    ],
    c_args: [
        platform_c_args
    ],
    dependencies: [
        platform_deps
    ]
)

libopenblt_dep = declare_dependency(
    include_directories: [
        platform_includes,
        include_directories('.')
    ],
    compile_args : [
        platform_c_args
    ],
    link_with: libopenblt.get_static_lib()
)

meson.override_dependency('libopenblt',libopenblt_dep)