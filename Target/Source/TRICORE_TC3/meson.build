if meson.get_compiler('c').get_id() != 'cctc'
error('Used compiler is not supported for this target')
endif

TRICORE_TC3_dep = declare_dependency(
    sources: [
        # Meson only supports GCC and clang, so IAR file is not used
        'GCC/cpu_comp.c',
        'can.c',
        'cpu.c',
        'flash.c',
        'mbrtu.c',
        'nvm.c',
        'rs232.c',
        'timer.c',
    ],
    include_directories: [
        '.'
    ]
)