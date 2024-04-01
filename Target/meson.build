openblt_core_dep = declare_dependency(
    sources: [
        'Source/asserts.c',
        'Source/backdoor.c',
        'Source/boot.c',
        'Source/com.c',
        'Source/cop.c',
        'Source/file.c',
        'Source/mb.c',
        'Source/net.c',
        'Source/xcp.c',
    ],
    include_directories: [
        'Source'
    ]
)

subdir('Source' / get_option('target'))
openblt_target_dep = get_variable(get_option('target') + '_dep')

ARMCM4_STM32F4_Nucleo_F446RE_bootloader = executable(
    'ARMCM4_STM32F4_Nucleo_F446RE_bootloader',
    name_suffix: 'elf',
    sources: [
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/App/app.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/App/hooks.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/App/led.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Core/Src/main.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Core/Src/stm32f4xx_hal_msp.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Core/Src/stm32f4xx_it.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Core/Src/syscalls.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Core/Src/sysmem.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Core/Src/system_stm32f4xx.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Core/Startup/startup_stm32f446retx.s',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c',
    ],
    include_directories: [
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/App',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Core/Inc',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Inc',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/CMSIS/Device/ST/STM32F4xx/Include',
        'Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/Drivers/CMSIS/Include',
    ],
    c_args: [
        '-DUSE_HAL_DRIVER',
        '-DSTM32F446xx',
        '-DUSE_FULL_LL_DRIVER',
    ],
    link_args: [
        '-T' + files('Demo/ARMCM4_STM32F4_Nucleo_F446RE_CubeIDE/Boot/STM32F446RETX_FLASH.ld')[0].full_path()
    ],
    dependencies: [
        openblt_core_dep,
        openblt_target_dep
    ]
)

bootloader_hex = custom_target(
    'ARMCM4_STM32F4_Nucleo_F446RE_bootloader_hex',
    input: [ARMCM4_STM32F4_Nucleo_F446RE_bootloader],
    output: ['ARMCM4_STM32F4_Nucleo_F446RE_bootloader.hex'],
    command: [find_program('objcopy'), '-O', 'ihex', '@INPUT@' , '@OUTPUT0@'],
    build_by_default: true
)

stlink = find_program('ST-LINK_CLI', disabler: true, required : false)

run_target(
    'flash_bootloader',
    command: [
        stlink, '-P', bootloader_hex
    ]
)