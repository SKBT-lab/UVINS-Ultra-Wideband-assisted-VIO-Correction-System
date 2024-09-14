#!/usr/bin/env python

import psutil
from datetime import datetime
import time

# 文件名
output_file_name = "/home/skbt/VIU_ROSBAG/memory_usage_log_Loop.txt"

def get_memory_usage():
    memory = psutil.virtual_memory()
    memory_usage_percentage = memory.percent
    return memory_usage_percentage

def main():
    # 初始化文件，写入表头
    with open(output_file_name, "w") as file:
        file.write("Timestamp MemoryUsage\n")

    start_time = time.time()

    try:
        while True:
            # 获取当前时间戳和内存占用率
            timestamp = time.time() - start_time
            memory_usage = get_memory_usage()
            print(memory_usage)

            # 将时间戳和内存占用百分比写入文件
            with open(output_file_name, "a") as file:
                file.write(f"{timestamp:.2f} {memory_usage:.2f}\n")

            # 休眠一段时间，例如1秒
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Program terminated by user.")

if __name__ == "__main__":
    main()