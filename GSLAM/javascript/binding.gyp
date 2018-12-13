{
  'targets': [
    {
      'target_name': 'binding',
      'includes': [
        'auto.gypi'
      ],
      'win_delay_load_hook': 'true',
      'sources': [
        'cpp/gslam.cpp'
      ],
      'include_dirs': [
        '<(module_root_dir)/../..'
      ],
      'cflags_cc!': [
        '-fno-rtti'
      ],
      'cflags_cc+': [
        '-std=c++11',
        '-frtti',
        '-Wno-deprecated-declarations',
        '-g'
      ],
      'msbuild_settings': {
        'ClCompile': {
          'AdditionalOptions': [
            # fatal error C1128: number of sections exceeded object file format limit:
            # compile with /bigobj
            '/bigobj'
          ]
        }
      },
      'conditions': [
        ['OS == "linux"', {
          'link_settings': {
            'libraries': [
            ]
          }
        }],
        ['OS == "win"', {
          'cflags_cc+': [
            '-stdlib=libc++'
          ],
          'link_settings': {
            'libraries': [
            ]
          }
        }]
      ]
    }
  ],
  'includes': [
    'auto-top.gypi'
  ]
}
