#!/usr/bin/env python

import psutil
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

# 文件名 运行前手动清空.dat，保留第一行
mmap_file_name = "memory_usage_mmap.dat"



def get_memory_usage():
    memory = psutil.virtual_memory()
    memory_usage_percentage = memory.percent
    return memory_usage_percentage

def initialize_mmap_file():
    # 读取一次内存占用百分比并写入内存映射文件
    memory_usage = get_memory_usage()
    with open(mmap_file_name, "w") as file:
        file.write(f"{memory_usage:.2f}\n")  # 初始化50个数据点，全为初始内存占用百分比

def update_memory_usage(frame):
    memory_usage = get_memory_usage()

    # 写入内存映射文件
    with open(mmap_file_name, "ab") as file:
        encoded_data = f"{memory_usage:.2f}\n".encode()
        os.lseek(file.fileno(), 0, os.SEEK_END)  # 将文件指针移动到文件的末尾
        os.write(file.fileno(), encoded_data)

    # 打印写入的信息
    print(f"Appended Memory Usage: {memory_usage:.2f}")

    # 读取内存映射文件中的数据
    with open(mmap_file_name, "r") as file:
        memory_usage_history = [float(line) for line in file]

    # 保持历史数据的长度，例如只保留最近的50个数据点
    # max_history_length = 500
    # memory_usage_history = memory_usage_history[-max_history_length:]
    # print(memory_usage_history)

    # 清空曲线图
    plt.gca().cla()

    # 绘制曲线图，从第50个数据点开始
    plt.plot(memory_usage_history, label='Memory Usage of Proposed Method(%)', color='red')
    plt.xlabel('Time(s)')
    plt.ylabel('Memory Usage(%)')
    plt.ylim(0, 100)
    plt.legend()

def main():
    initialize_mmap_file()
    # 设置动画更新间隔（毫秒）
    update_interval = 1000

    # 使用FuncAnimation创建动画
    ani = FuncAnimation(plt.gcf(), update_memory_usage, interval=update_interval)

    plt.show()

if __name__ == "__main__":
    main()