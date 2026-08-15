"""远程 ZIP64 单文件提取器 v2 —— 两阶段 + 断点续传

阶段A: 用 HTTP Range 下载目标文件的压缩数据段到 <name>.deflate
       （sidecar 文件 <name>.deflate.progress 记录已下载字节数，可安全断点续传）
阶段B: 本地 zlib 解压 deflate 流 → 最终 .ply
"""
import struct, zlib, os, sys, time

URL = "https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/datasets/pretrained/models.zip"
ZIP_SIZE = 14660630999          # 已知总大小
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")

def fetch_range(start, end, retries=8):
    """下载 [start, end] 闭区间字节，带重试与退避"""
    import urllib.request
    for i in range(retries):
        try:
            req = urllib.request.Request(URL, headers={'Range': f'bytes={start}-{end}', 'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(req, timeout=180) as r:
                data = r.read()
            if len(data) != end - start + 1:
                raise IOError(f"short read: {len(data)} != {end-start+1}")
            return data
        except Exception as e:
            if i == retries - 1:
                raise
            wait = 5 * (i + 1)
            print(f"  [retry {i+1}] {e} → 等待 {wait}s...")
            time.sleep(wait)

def main():
    LIST_ONLY = '--list' in sys.argv
    os.makedirs(OUT_DIR, exist_ok=True)
    print("[1/5] 下载 zip 尾部 512KB (含中央目录)...")
    TAIL = 512 * 1024
    tail = fetch_range(ZIP_SIZE - TAIL, ZIP_SIZE - 1)

    print("[2/5] 解析 ZIP64 定位中央目录...")
    loc_idx = tail.rfind(b'\x50\x4b\x06\x07')          # ZIP64 EOCD locator
    assert loc_idx >= 0, "ZIP64 locator not found"
    z64_eocd_off = struct.unpack('<Q', tail[loc_idx+8:loc_idx+16])[0]
    z64 = fetch_range(z64_eocd_off, z64_eocd_off + 64)
    cd_size = struct.unpack('<Q', z64[40:48])[0]
    cd_off  = struct.unpack('<Q', z64[48:56])[0]
    print(f"  central directory: offset={cd_off}, size={cd_size}")

    print("[3/5] 下载中央目录并找 truck 的 PLY...")
    cd = fetch_range(cd_off, cd_off + cd_size - 1)
    targets = []
    pos = 0
    while pos + 46 <= len(cd):
        if cd[pos:pos+4] != b'\x50\x4b\x01\x02':
            break
        method      = struct.unpack('<H', cd[pos+10:pos+12])[0]
        comp_size   = struct.unpack('<I', cd[pos+20:pos+24])[0]
        uncomp_size = struct.unpack('<I', cd[pos+24:pos+28])[0]
        name_len    = struct.unpack('<H', cd[pos+28:pos+30])[0]
        extra_len   = struct.unpack('<H', cd[pos+30:pos+32])[0]
        comment_len = struct.unpack('<H', cd[pos+32:pos+34])[0]
        local_off   = struct.unpack('<I', cd[pos+42:pos+46])[0]
        name = cd[pos+46:pos+46+name_len].decode('utf-8', 'replace')
        extra = cd[pos+46+name_len:pos+46+name_len+extra_len]
        if comp_size == 0xFFFFFFFF or uncomp_size == 0xFFFFFFFF or local_off == 0xFFFFFFFF:
            p = 0
            while p + 4 <= len(extra):
                eid, esz = struct.unpack('<HH', extra[p:p+4])
                if eid == 0x0001:
                    ed = extra[p+4:p+4+esz]
                    q = 0
                    if uncomp_size == 0xFFFFFFFF:
                        uncomp_size = struct.unpack('<Q', ed[q:q+8])[0]; q += 8
                    if comp_size == 0xFFFFFFFF:
                        comp_size = struct.unpack('<Q', ed[q:q+8])[0]; q += 8
                    if local_off == 0xFFFFFFFF:
                        local_off = struct.unpack('<Q', ed[q:q+8])[0]
                p += 4 + esz
        if name.lower().endswith('.ply'):
            targets.append((name, method, comp_size, uncomp_size, local_off))
        pos += 46 + name_len + extra_len + comment_len

    if not targets:
        print("ERROR: 未找到任何 PLY，退出")
        sys.exit(1)

    if LIST_ONLY:
        print(f"\n共 {len(targets)} 个 PLY，'--list' 模式结束")
        sys.exit(0)

    # 优先 truck 的最终模型(iteration_30000 + point_cloud)
    chosen = None
    for t in targets:
        p = t[0].lower()
        if 'truck' in p and 'point_cloud' in p and 'iteration_30000' in p:
            chosen = t
            break
    if chosen is None:
        chosen = targets[0]
    name, method, comp_size, uncomp_size, local_off = chosen
    if method != 8:
        print(f"ERROR: 目标压缩方式 {method}，脚本只支持 deflate(8)")
        sys.exit(1)
    print(f"\n[4/5] 目标: {name}  comp={comp_size/1e6:.0f}MB → uncomp={uncomp_size/1e6:.0f}MB")

    # 读 local file header (30B) 确认数据起点
    lh = fetch_range(local_off, local_off + 29)
    lh_name_len  = struct.unpack('<H', lh[26:28])[0]
    lh_extra_len = struct.unpack('<H', lh[28:30])[0]
    data_start = local_off + 30 + lh_name_len + lh_extra_len

    parts = [x for x in name.split('/') if x not in ('point_cloud', 'input')]
    base = '_'.join(parts).replace('.ply', '')
    raw_path  = os.path.join(OUT_DIR, base + '.deflate')        # 阶段A产物
    prog_path = raw_path + '.progress'                           # 进度 sidecar
    out_path  = os.path.join(OUT_DIR, base + '.ply')             # 阶段B产物

    # ========== 阶段A：下载压缩数据段（可断点续传） ==========
    done = 0
    if os.path.exists(prog_path):
        done = int(open(prog_path).read().strip())
    if os.path.exists(out_path) and os.path.getsize(out_path) == uncomp_size:
        print(f"  最终文件已存在且完整（{uncomp_size/1e6:.0f}MB），跳过下载")
        sys.exit(0)
    print(f"  压缩段进度: {done/1e6:.0f}MB / {comp_size/1e6:.0f}MB")
    t0 = time.time()
    with open(raw_path, 'ab') as f:
        CHUNK = 8 * 1024 * 1024
        while done < comp_size:
            n = min(CHUNK, comp_size - done)
            raw = fetch_range(data_start + done, data_start + done + n - 1)
            f.write(raw)
            done += n
            open(prog_path, 'w').write(str(done))   # 每块更新进度
            speed = done / 1e6 / max(time.time() - t0, 0.01)
            print(f"  {done/1e6:.0f}/{comp_size/1e6:.0f}MB  ({speed:.1f} MB/s)")
    print(f"[4.5] 压缩段下载完成: {raw_path}")

    # ========== 阶段B：本地解压 ==========
    print("[5/5] 本地解压 deflate 流...")
    t0 = time.time()
    d = zlib.decompressobj(-zlib.MAX_WBITS)
    with open(raw_path, 'rb') as fin, open(out_path, 'wb') as fout:
        while True:
            chunk = fin.read(16 * 1024 * 1024)
            if not chunk:
                break
            fout.write(d.decompress(chunk))
        fout.write(d.flush())
    os.remove(raw_path)
    os.remove(prog_path)
    print(f"  完成 → {out_path}  ({os.path.getsize(out_path)/1e6:.0f}MB, 用时 {time.time()-t0:.0f}s)")

if __name__ == '__main__':
    main()
