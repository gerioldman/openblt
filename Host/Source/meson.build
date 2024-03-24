project(
    'libopenblt',
    'c',
    version: '0.0.0',
    default_options: {
        'c_std': 'gnu99'
    }
)

subdir('LibOpenBLT')
subdir('SeedNKey')

executable(
    'bootcommander',
    sources: [
        'BootCommander/main.c'
    ],
    dependencies: [
        dependency('libopenblt')
    ]
)