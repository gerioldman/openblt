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

if get_option('build_demo') and get_option('target') == 'ARMCM4_STM32F4'

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

endif

if get_option('build_demo') and get_option('target') == 'RISCV_CH32V30x'

CH32V307V_EVT_R1_bootloader = executable(
    'CH32V307V_EVT_R1_GCC_bootloader',
    name_suffix: 'elf',
    sources: [
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Core/core_riscv.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Debug/debug.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_adc.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_bkp.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_can.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_crc.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_dac.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_dbgmcu.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_dma.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_dvp.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_eth.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_exti.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_flash.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_fsmc.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_gpio.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_i2c.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_iwdg.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_misc.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_opa.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_pwr.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_rcc.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_rng.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_rtc.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_sdio.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_spi.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_tim.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_usart.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/src/ch32v30x_wwdg.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Startup/startup_ch32v30x_D8C.S',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/User/ch32v30x_it.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/User/main.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/User/system_ch32v30x.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/User/hooks.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/User/led.c',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/User/usb_descriptors.c',
    ],
    include_directories: [
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Core',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Debug',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/Peripheral/inc',
        'Demo/CH32V307V-EVT-R1_GCC/Boot/User',
    ],
    c_args: [
        '-DCFG_TUSB_MCU=OPT_MCU_CH32V307'
    ],
    link_args: [
        '-T' + files('Demo/CH32V307V-EVT-R1_GCC/Boot/Ld/Link.ld')[0].full_path(),
        '-Wl,-Map=' + meson.current_build_dir() / 'CH32V307V_EVT_R1_GCC_bootloader.map'
    ],
    dependencies: [
        openblt_core_dep,
        openblt_target_dep,
        dependency('tinyusb')
    ]
)

wlink = find_program('wlink', disabler: true, required : false)

run_target(
    'flash_bootloader',
    command: [
        wlink, 'flash', CH32V307V_EVT_R1_bootloader
    ]
)

endif