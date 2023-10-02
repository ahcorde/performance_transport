import os

import matplotlib.pyplot as plt
import numpy as np

import matplotlib.animation as animation

import pandas as pd

folder_name = '/home/ahcorde/Downloads/transport_data/'

transport_hint = ['raw', 'compressed']
compressed_types = ['jpeg', 'png', 'zstd']
jpeg_compress = ['90', '70', '40', '30', '10']
png_compress = ['1', '3', '5', '7', '9']
zstd_compress = ['1', '3', '5', '7', '9']
image_sizes = ['4096', '2048', '1024', '512']

jpeg_90_cpu_mem = {}
jpeg_90 = {}

png_90_cpu_mem = {}
png_90 = {}

zstd_90_cpu_mem = {}
zstd_90 = {}

subscriber_data_cpu_mem = os.path.join(folder_name, 'subscriber_data_cpu_mem')
subscriber_data = os.path.join(folder_name, 'subscriber_data')

for transport in transport_hint:
    for size in image_sizes:
        name = '_' + str(size) + '_' + str(size) + '_' + transport
        if transport == 'compressed':
            for compress_type in compressed_types:
                if compress_type == 'jpeg':
                    for compress in jpeg_compress:
                        name2 = name + '_' + compress_type + '_' + compress + '.csv'
                        if compress == '90':
                            jpeg_90_cpu_mem[subscriber_data_cpu_mem + name2] = {'size' : size}
                            jpeg_90[subscriber_data + name2] = {'size' : size}
                        # print(subscriber_data_cpu_mem + name2)
                if compress_type == 'png':
                    for compress in png_compress:
                        name2 = name + '_' + compress_type + '_' + compress + '.csv'
                        if compress == '9':
                            png_90_cpu_mem[subscriber_data_cpu_mem + name2] = {'size' : size}
                            png_90[subscriber_data + name2] = {'size' : size}
                            print(subscriber_data + name2)
                if compress_type == 'zstd':
                    for compress in png_compress:
                        name2 = name + '_' + compress_type + '_' + compress + '.csv'
                        if compress == '9':
                            zstd_90_cpu_mem[subscriber_data_cpu_mem + name2] = {'size' : size}
                            zstd_90[subscriber_data + name2] = {'size' : size}
                            print(subscriber_data + name2)
# print(jpeg_90)
# print(png_90)

# columns = ['timestamp', 'uptime', 'cpuusage', 'memory', 'anonmemory', 'vm', 'rmbytes', 'tmbytes', 'rpackets', 'tpackets']

# for column in columns:
#     if column == 'timestamp':
#         continue
#     fig, ax = plt.subplots()
#     for compress_90, value in jpeg_90_cpu_mem.items():
#         df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
#         df['timestamp'] = df['timestamp'] - df['timestamp'][0]
#         ax.plot(df['timestamp'].values, df[column].values, label=f"{column} {value['size']}x{value['size']}")
#         ax.set_title(column)
#         ax.set_ylabel(column)
#         ax.set_xlabel('Time [s]')
#         ax.legend()

columns = ['fps', 'response_t']

for column in columns:
    if column == 'timestamp':
        continue
    fig, ax = plt.subplots()
    for compress_90, value in jpeg_90.items():
        print(compress_90)
        df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
        x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
        ax.plot(x, df[column].values, label=f"JPEG {column} {value['size']}x{value['size']}")
    for compress_90, value in png_90.items():
        print(compress_90)
        df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
        x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
        ax.plot(x, df[column].values, label=f"PNG {column} {value['size']}x{value['size']}")
    for compress_90, value in zstd_90.items():
        print(compress_90)
        df = pd.read_csv(compress_90, delimiter=',', names=columns, header=0, skiprows=0)
        x = np.linspace(0, 300, df['fps'].values.size, endpoint=False)
        ax.plot(x, df[column].values, label=f"ZSTD {column} {value['size']}x{value['size']}")
    ax.set_title(column)
    ax.set_ylabel(column)
    ax.set_xlabel('Time [s]')
    ax.legend()


# columns = ['translation.x', 'translation.y', 'translation.z', 'rotation.x', 'rotation.y', 'rotation.z', 'rotation.w']
# df = pd.read_csv('data_ground_truth.csv', names=columns, delimiter=',', header=None)
# df_slam = pd.read_csv('slam.csv', names=columns, delimiter=',', header=None)

# scat = ax.scatter(df_slam['translation.x'][0], df_slam['translation.x'][1], c='b', s=5,  label='slam')
# line2 = ax.plot(df['translation.x'][0], df['translation.x'][1], label=f'ground_truth')[0]

# ax.set(xlim=[0, 20], ylim=[-5, 5], xlabel='X [m]', ylabel='Y [m]')
# ax.legend()

# def update(frame):
#     # for each frame, update the data stored on each artist.
#     x = df_slam['translation.x'][:frame]
#     y = df_slam['translation.y'][:frame]
#     # update the scatter plot:
#     data = np.stack([x, y]).T
#     scat.set_offsets(data)
#     # update the line plot:
#     line2.set_xdata(df['translation.x'][:frame])
#     line2.set_ydata(df['translation.y'][:frame])
#     return (scat, line2)

# ani = animation.FuncAnimation(fig=fig, func=update, frames=504, interval=30)
plt.show()
