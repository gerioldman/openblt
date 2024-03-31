if meson.get_compiler('c').get_id() != 'gcc' and meson.get_compiler('c').get_id() != 'armclang'
error('Used compiler is not supported for this target')
endif

ARMCM3_EFM32_dep = declare_dependency(
    sources: [
        # Meson only supports GCC and clang, so IAR file is not used
        'GCC/cpu_comp.c',
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