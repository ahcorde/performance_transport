import os

import matplotlib.pyplot as plt
import numpy as np

import matplotlib.animation as animation

import pandas as pd

folder_name = '/home/ahcorde/Downloads/transport_data/'

transport_hint = ['raw', 'draco', 'zstd', 'zlib']
compress_array = ['3', '6', '9']
sizes = ['75260', '2762400', '1179084']

data_cpu_mem = {}
data = {}


subscriber_data_cpu_mem = os.path.join(folder_name, 'subscriber_point_cloud_data_cpu_mem')
subscriber_data = os.path.join(folder_name, 'subscriber_point_cloud_data')

for transport in transport_hint:
    for size in sizes:
        name = '_' + str(size) + '_' + transport
        name2 = ''
        if transport == 'raw':
            name2 =  name + '.csv'
            data_cpu_mem[subscriber_data_cpu_mem + name2] = {'size' : size, 'transport': transport, 'compress': ''}
            data[subscriber_data + name2] = {'size' : size, 'transport': transport, 'compress': ''}
            print(subscriber_data + name2)
        else:
            for compress in compress_array:
                name2 =  name + '_' + compress + '.csv'
                data_cpu_mem[subscriber_data_cpu_mem + name2] = {'size' : size, 'transport': transport, 'compress': compress}
                data[subscriber_data + name2] = {'size' : size, 'transport': transport, 'compress': compress}
                print(subscriber_data + name2)

columns = ['fps', 'response_t']
for column in columns:
    fig, ax = plt.subplots()
    for filename, value in data.items():
        if value['size'] == '2762400':
            try:
                df = pd.read_csv(filename, delimiter=',', names=columns, header=0, skiprows=0)
                x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
                ax.plot(x, df[column].values, label=f"{value['transport']} {value['compress']} {column} {value['size']}x{value['size']}")
            except:
                pass
    ax.set_title(column)
    ax.set_ylabel(column)
    ax.set_xlabel('Time [s]')
    ax.legend()
plt.show()
