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

# transport_hint = ['raw', 'draco', 'zstd', 'zlib']
transport_hint = ['zlib', 'zstd', 'draco']
compress_array = ['3', '6', '9']
sizes = ['indoor', 'outdoor']

color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
linetype = ['-', '--', '-.', ':', 'solid', 'dashed', 'dashdot', 'dotted']


for type_node in ['publisher', 'subscriber']:
    data_cpu_mem_str = os.path.join(folder_name, type_node + '_point_cloud_data_cpu_mem')
    data_str = os.path.join(folder_name, type_node + '_point_cloud_data')

    data_cpu_mem = {}
    data = {}

    for transport in transport_hint:
        for size in sizes:
            name = '_' + str(size) + '_' + transport
            name2 = ''
            if transport == 'raw':
                name2 = name + '.csv'
                data_cpu_mem[data_cpu_mem_str + name2] = {'size': size,
                                                          'transport': transport,
                                                          'compress': ''}
                data[data_str + name2] = {'size': size,
                                          'transport': transport,
                                          'compress': ''}
                print(data_str + name2)
            else:
                for compress in compress_array:
                    name2 = name + '_' + compress + '.csv'
                    data_cpu_mem[data_cpu_mem_str + name2] = {'size': size,
                                                              'transport': transport,
                                                              'compress': compress}
                    data[data_str + name2] = {'size': size,
                                              'transport': transport,
                                              'compress': compress}
                    print(data_str + name2)

    columns = ['timestamp', 'uptime', 'cpuusage', 'memory', 'anonmemory',
               'vm', 'rmbytes', 'tmbytes', 'rpackets', 'tpackets']

    for size in sizes:
        for column in columns:
            if column == 'timestamp' or column == 'uptime':
                continue
            fig, ax = plt.subplots()

            color_index = 0
            linetype_index = 0
            for filename, value in data_cpu_mem.items():
                df = pd.read_csv(filename, delimiter=',', names=columns, header=0, skiprows=0)
                df['timestamp'] = df['timestamp'] - df['timestamp'][0]
                ax.plot(df['timestamp'].values, df[column].values, linestyle=linetype[linetype_index],
                        color=color[color_index],
                        label=f"{value['transport']} {value['compress']} {column} {value['size']}")
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

    for size in sizes:
        for column in columns:
            color_index = 0
            linetype_index = 0

            fig, ax = plt.subplots()
            for filename, value in data.items():
                print('filename ', filename)
                print('value ', value)
                if value['size'] == size:
                    print(filename, columns)
                    print(type_node)
                    df = pd.read_csv(filename, delimiter=',',
                                     names=columns, header=0, skiprows=0)
                    x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
                    transport = value['transport']
                    ax.plot(x, df[column].values,
                            label=f"{transport} {value['compress']} {column} {value['size']}",
                            color=color[color_index],
                            linestyle=linetype[linetype_index])
                    linetype_index = (linetype_index + 1) % 8
                    color_index = (color_index + 1) % 7

            ax.set_title(type_node + ' ' + column + ' ' + size)
            ax.set_ylabel(column)
            ax.set_xlabel('Time [s]')
            ax.legend()
plt.show()
