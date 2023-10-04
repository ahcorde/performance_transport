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

import matplotlib.pyplot as plt
import numpy as np

import pandas as pd

folder_name = '/home/ahcorde/TRI/performance_transport_ws/build/performance_transport/test'

transport_hint = ['raw', 'draco', 'zstd', 'zlib']
compress_array = ['3', '6', '9']
sizes = ['75260', '2762400', '1179084']

data_cpu_mem = {}
data = {}

color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
linetype = ['-', '--', '-.', ':', 'solid', 'dashed', 'dashdot', 'dotted']

color_index = 0
linetype_index = 0

for type_node in ['publisher', 'subscriber']:
    data_cpu_mem_str = os.path.join(folder_name, type_node + '_point_cloud_data_cpu_mem')
    data_str = os.path.join(folder_name, type_node + '_point_cloud_data')

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

    if type_node == 'publisher':
        columns = ['fps']
    else:
        columns = ['fps', 'response_t']

    for size in sizes:
        for column in columns:
            fig, ax = plt.subplots()
            for filename, value in data.items():
                print('filename ', filename)
                print('value ', value)
                if value['size'] == size:
                    df = pd.read_csv(filename, delimiter=',',
                                     names=columns, header=0, skiprows=0)
                    x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
                    transport = value['transport']
                    ax.plot(x, df[column].values,
                            label=f"{transport} {value['compress']} {column} {value['size']}")
            ax.set_title(type_node + ' ' + column + ' ' + size)
            ax.set_ylabel(column)
            ax.set_xlabel('Time [s]')
            ax.legend()
plt.show()
