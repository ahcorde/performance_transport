# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import matplotlib.pyplot as plt
import numpy as np

import pandas as pd

folder_name = sys.argv[1]

transport_hint = ['raw', 'compressed', 'zstd']
compressed_types = ['jpeg', 'png']

transport_hint = ['avif']
compressed_types = ['avf']

jpeg_compress = ['90', '50', '20']
png_compress = ['3', '6', '9']
zstd_compress = ['3', '6', '9']
avif_compress = ['90', '50', '20']
image_sizes = ['4096', '2048', '1024', '512']

jpeg_compress = ['90']
image_sizes = ['4096', '2048', '1024', '512']

color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
linetype = ['-', '--', '-.', ':', 'solid', 'dashed', 'dashdot', 'dotted']

color_index = 0
linetype_index = 0

for type_node in ['publisher', 'subscriber']:
    data_cpu_mem_str = os.path.join(folder_name, type_node + '_data_cpu_mem')
    data_str = os.path.join(folder_name, type_node + '_data')

    raw_cpu_mem = {}
    raw = {}

    jpeg_90_cpu_mem = {}
    jpeg_90 = {}

    png_90_cpu_mem = {}
    png_90 = {}

    zstd_90_cpu_mem = {}
    zstd_90 = {}

    avif_cpu_mem = {}
    avif = {}

    for transport in transport_hint:
        for size in image_sizes:
            name = '_' + str(size) + '_' + str(size) + '_' + transport
            if transport == 'raw':
                raw_cpu_mem[data_cpu_mem_str + name + '.csv'] = {'size': size}
                raw[data_str + name + '.csv'] = {'size': size}
            elif transport == 'compressed':
                for compress_type in compressed_types:
                    if compress_type == 'jpeg':
                        for compress in jpeg_compress:
                            name2 = name + '_' + compress_type + '_' + compress + '.csv'
                            jpeg_90_cpu_mem[data_cpu_mem_str + name2] = {'size': size, 'compress': compress}
                            jpeg_90[data_str + name2] = {'size': size, 'compress': compress}
                            # print(data_cpu_mem_str + name2)
                    if compress_type == 'png':
                        for compress in png_compress:
                            name2 = name + '_' + compress_type + '_' + compress + '.csv'
                            png_90_cpu_mem[data_cpu_mem_str + name2] = {'size': size, 'compress': compress}
                            png_90[data_str + name2] = {'size': size, 'compress': compress}
                                # print(data_str + name2)
            elif transport == 'zstd':
                for compress in png_compress:
                    name2 = name + '_' + compress + '.csv'
                    zstd_90_cpu_mem[data_cpu_mem_str + name2] = {'size': size, 'compress': compress}
                    zstd_90[data_str + name2] = {'size': size, 'compress': compress}
            elif transport == 'avif':
                for compress in avif_compress:
                    name2 = name + '_' + compress + '.csv'
                    avif_cpu_mem[data_cpu_mem_str + name2] = {'size': size, 'compress': compress}
                    avif[data_str + name2] = {'size': size, 'compress': compress}
                        # print(data_str + name2)

    columns = ['timestamp', 'uptime', 'cpuusage', 'memory', 'anonmemory',
               'vm', 'rmbytes', 'tmbytes', 'rpackets', 'tpackets']

    for column in columns:
        if column == 'timestamp' or column == 'uptime':
            continue
        fig, ax = plt.subplots()

        for compress_90, value in raw_cpu_mem.items():
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            df['timestamp'] = df['timestamp'] - df['timestamp'][0]
            ax.plot(df['timestamp'].values, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"RAW {column} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
        color_index = color_index + 1
        for compress_90, value in jpeg_90_cpu_mem.items():
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            df['timestamp'] = df['timestamp'] - df['timestamp'][0]
            ax.plot(df['timestamp'].values, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"JPEG {column} {value['compress']} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
            color_index = (color_index + 1) % 7
        for compress_90, value in png_90_cpu_mem.items():
            print(compress_90)
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            df['timestamp'] = df['timestamp'] - df['timestamp'][0]
            print(linetype[linetype_index])
            print('color_index ', color_index)
            print(color[color_index])
            ax.plot(df['timestamp'].values, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"PNG {column} {value['compress']} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
            color_index = (color_index + 1) % 7
        for compress_90, value in zstd_90_cpu_mem.items():
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            df['timestamp'] = df['timestamp'] - df['timestamp'][0]
            ax.plot(df['timestamp'].values, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"ZSTD {column} {value['compress']} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
            color_index = (color_index + 1) % 7
        for compress_90, value in avif_cpu_mem.items():
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            df['timestamp'] = df['timestamp'] - df['timestamp'][0]
            ax.plot(df['timestamp'].values, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"AVIF {column} {value['compress']} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
            color_index = (color_index + 1) % 7

        ax.set_title(type_node + " " + column)
        ax.set_ylabel(column)
        ax.set_xlabel('Time [s]')
        ax.legend()

    if type_node == 'publisher':
        columns = ['fps']
    else:
        columns = ['fps', 'response_t']

    color_index = 0
    linetype_index = 0

    for column in columns:
        if column == 'timestamp':
            continue

        fig, ax = plt.subplots()
        for raw_file, value in raw.items():
            print(raw_file)
            df = pd.read_csv(raw_file, delimiter=',', names=columns, header=0, skiprows=0)
            x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
            ax.plot(x, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"RAW {column} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8

        color_index = color_index + 1
        for compress_90, value in jpeg_90.items():
            print(compress_90)
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
            ax.plot(x, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"JPEG {column} {value['compress']} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
            color_index = (color_index + 1) % 7
        for compress_90, value in png_90.items():
            print(compress_90)
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
            ax.plot(x, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"PNG {column} {value['compress']} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
            color_index = (color_index + 1) % 7
        for compress_90, value in zstd_90.items():
            print(compress_90)
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
            ax.plot(x, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"ZSTD {column} {value['compress']} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
            color_index = (color_index + 1) % 7
        for compress_90, value in avif.items():
            print(compress_90)
            df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
            x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
            ax.plot(x, df[column].values, linestyle=linetype[linetype_index],
                    color=color[color_index],
                    label=f"AVIF {column} {value['compress']} {value['size']}x{value['size']}")
            linetype_index = (linetype_index + 1) % 8
            color_index = (color_index + 1) % 7
        ax.set_title(type_node + " " + column)
        ax.set_ylabel(column)
        ax.set_xlabel('Time [s]')
        ax.set_title(f'{type_node} {column}')
        ax.legend()
plt.show()
