if meson.get_compiler('c').get_id() != 'gcc' and meson.get_compiler('c').get_id() != 'armclang'
error('Used compiler is not supported for this target')
endif

ARMCM4_S32K14_dep = declare_dependency(
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