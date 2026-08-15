# -*- coding: utf-8 -*-
"""
生成 PLY 冒烟数据：球面点云（Fibonacci 球均匀分布）
- 4096 点，球心原点，半径 1
- 颜色按高度渐变：y=-1 蓝 → y=+1 红（用于视觉验证）
- 输出两个版本：ASCII + binary_little_endian（互验解析器两条路线）
用法: python generate_smoke_ply.py <输出目录>
"""
import math
import os
import struct
import sys

N = 4096          # 点数
RADIUS = 1.0      # 球半径

def build_points():
    """Fibonacci 球面：点在球面上近似均匀分布"""
    pts = []
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))  # 黄金角 ≈ 2.399963
    for i in range(N):
        y = 1.0 - (2.0 * i + 1.0) / N                 # y 从 1 到 -1 均匀取
        r = math.sqrt(max(0.0, 1.0 - y * y))          # 当前纬度的半径
        theta = golden_angle * i
        pts.append((math.cos(theta) * r * RADIUS, y * RADIUS, math.sin(theta) * r * RADIUS))
    return pts

def color_for(y):
    """按高度渐变：蓝(-1) → 红(+1)"""
    t = (y + 1.0) / 2.0
    return (int(round(255 * t)), 0, int(round(255 * (1.0 - t))))

HEADER = (
    "ply\n"
    "format {fmt}\n"
    "element vertex {n}\n"
    "property float x\n"
    "property float y\n"
    "property float z\n"
    "property uchar red\n"
    "property uchar green\n"
    "property uchar blue\n"
    "end_header\n"
)

def write_ascii(path, pts):
    with open(path, "w", encoding="ascii") as f:
        f.write(HEADER.format(fmt="ascii 1.0", n=N))
        for (x, y, z) in pts:
            r, g, b = color_for(y)
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n")
    print(f"[OK] {path}  ({os.path.getsize(path)} bytes)")

def write_binary(path, pts):
    with open(path, "wb") as f:
        f.write(HEADER.format(fmt="binary_little_endian 1.0", n=N).encode("ascii"))
        for (x, y, z) in pts:
            r, g, b = color_for(y)
            f.write(struct.pack("<fffBBB", x, y, z, r, g, b))  # 15 bytes/point
    print(f"[OK] {path}  ({os.path.getsize(path)} bytes)")

if __name__ == "__main__":
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)
    pts = build_points()
    write_ascii(os.path.join(out_dir, "smoke_ascii.ply"), pts)
    write_binary(os.path.join(out_dir, "smoke_binary.ply"), pts)
    print(f"共 {N} 点，球心原点半径 {RADIUS}，包围盒应为 [-1,-1,-1] ~ [1,1,1]")
