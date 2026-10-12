#!/usr/bin/env python3
"""
Nokov/XINGYING VRPN 原始数据诊断工具
=====================================
直接连接 VRPN TCP 端口 (默认 3883)，解析 VRPN 协议并打印：
  - 发现的 tracker 名称
  - 位置/姿态数据 (含数值量级判断单位)

用法:
  python3 vrpn_raw_diagnose.py --host 10.1.1.198
  python3 vrpn_raw_diagnose.py --host 10.1.1.198 --port 3883 --duration 30

VRPN 协议要点:
  - TCP 连接，服务端主动推送数据
  - 每条消息: header(24 bytes) + payload
  - header: type(4) + sender(4) + sec(4) + usec(4) + len(4) + padding(4)
  - tracker 名称在 sender 消息中以 ASCII 形式出现
  - position 消息: type=1, 包含 (x,y,z) 各 8 bytes (double)
  - quaternion 消息: type=3, 包含 (qx,qy,qz,qw) 各 8 bytes (double)
"""

import socket
import struct
import argparse
import time
import sys
from collections import defaultdict


# VRPN message types (low-level)
VRPN_CONNECTION = 0
VRPN_SENDER_DESCRIPTION = -1  # 0xFFFFFFFF
VRPN_TYPE_MESSAGE = -2         # 0xFFFFFFFE

# VRPN tracker message subtypes
TRACKER_POSITION = 1
TRACKER_VELOCITY = 2
TRACKER_ACCELERATION = 3
TRACKER_QUATERNION = 4  # Not standard but used by some implementations
TRACKER_MASK = 0x0000FFFF

# Known sender IDs mapping (populated as we discover them)
# sender_id -> {'name': str, 'type': str, 'messages': []}


def parse_vrpn_header(data: bytes) -> dict:
    """Parse a VRPN message header (24 bytes).

    struct timeval {
        int32_t tv_sec;
        int32_t tv_usec;
    };

    struct vrpn_FRAMEWORK_HEADER {
        int32_t  type;       // message type
        int32_t  sender_id;  // sender identifier
        timeval  msg_time;   // timestamp
        int32_t  payload_len;// length of payload
        int32_t  padding;    // alignment
    };
    """
    if len(data) < 24:
        return None
    # > = big-endian, i = int32
    msg_type, sender_id, tv_sec, tv_usec, payload_len, padding = struct.unpack(
        '>iiiiii', data[:24])
    return {
        'type': msg_type,
        'sender_id': sender_id,
        'tv_sec': tv_sec,
        'tv_usec': tv_usec,
        'payload_len': payload_len,
        'padding': padding,
    }


def try_parse_tracker_pos(payload: bytes) -> dict:
    """Try to parse tracker position payload. Returns None if doesn't look right."""
    if len(payload) < 24:  # 3 doubles = 24 bytes minimum
        return None
    try:
        x, y, z = struct.unpack('>ddd', payload[:24])
        # Sanity check: values should be within reasonable bounds for motion capture
        # (-100000 to 100000 mm, or -100 to 100 m)
        if any(abs(v) > 1e8 or abs(v) < 1e-15 for v in [x, y, z]):
            return None
        return {'x': x, 'y': y, 'z': z}
    except struct.error:
        return None


def try_parse_tracker_quat(payload: bytes) -> dict:
    """Try to parse tracker quaternion payload."""
    if len(payload) < 48:
        return None
    try:
        qx, qy, qz, qw = struct.unpack('>dddd', payload[:32])
        # quaternion sanity check
        norm = qx**2 + qy**2 + qz**2 + qw**2
        if not (0.5 < norm < 1.5):
            return None
        return {'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}
    except struct.error:
        return None


def try_parse_sender_description(payload: bytes) -> str:
    """Try to parse a sender description message (ASCII text)."""
    try:
        # The sender description usually contains the sender name as ASCII
        text = payload.decode('ascii', errors='ignore')
        # Filter out non-printable characters
        clean = ''.join(c for c in text if 32 <= ord(c) < 127)
        if len(clean) > 2:
            return clean
    except Exception:
        pass
    return None


def decode_vrpn_type(msg_type: int) -> str:
    """Decode VRPN message type to human-readable string."""
    if msg_type == -1:
        return 'SENDER_DESC'
    if msg_type == -2:
        return 'TYPE_MESSAGE'

    # High 16 bits: service type, low 16 bits: message subtype
    svc_type = (msg_type >> 16) & 0xFFFF
    svc_subtype = msg_type & 0xFFFF

    svc_names = {
        1: 'tracker',
        2: 'analog',
        3: 'button',
        4: 'dial',
        5: 'force_device',
        6: 'sound_server',
        7: 'text',
        8: 'custom',
    }
    svc_name = svc_names.get(svc_type, f'svc_{svc_type}')
    return f'{svc_name}({svc_subtype})'


def main():
    parser = argparse.ArgumentParser(description='Nokov VRPN Raw Diagnostic')
    parser.add_argument('--host', default='10.1.1.198', help='VRPN server IP')
    parser.add_argument('--port', type=int, default=3883, help='VRPN port')
    parser.add_argument('--duration', type=int, default=30, help='Capture duration (seconds)')
    parser.add_argument('--timeout', type=float, default=5.0, help='Connection timeout')
    args = parser.parse_args()

    print('=' * 70)
    print(f'🔍 Nokov/XINGYING VRPN 原始数据诊断')
    print(f'   服务器: {args.host}:{args.port}')
    print(f'   采集时长: {args.duration}s')
    print('=' * 70)

    # Connect
    print(f'\n⏳ 正在连接 {args.host}:{args.port}...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(args.timeout)

    try:
        sock.connect((args.host, args.port))
        print('✅ TCP 连接成功！等待 VRPN 数据...\n')
    except Exception as e:
        print(f'❌ 连接失败: {e}')
        return 1

    # Stats
    senders = {}  # sender_id -> {'name': str, 'service_types': set()}
    tracker_data = defaultdict(list)  # sender_id -> [(x,y,z, qx,qy,qz,qw), ...]
    msg_count = 0
    pos_count = 0
    quat_count = 0
    start_time = time.time()
    buffer = b''

    # Set short timeout for recv so we can check duration
    sock.settimeout(0.5)

    try:
        while time.time() - start_time < args.duration:
            try:
                chunk = sock.recv(8192)
                if not chunk:
                    print('⚠️  服务端关闭了连接')
                    break
                buffer += chunk
            except socket.timeout:
                continue

            # Parse as many messages as possible from buffer
            while len(buffer) >= 24:
                header = parse_vrpn_header(buffer[:24])
                if header is None:
                    buffer = buffer[1:]  # skip one byte and retry
                    continue

                payload_len = header['payload_len']
                total_len = 24 + payload_len

                if len(buffer) < total_len:
                    break  # need more data

                payload = buffer[24:total_len]
                buffer = buffer[total_len:]
                msg_count += 1

                sid = header['sender_id']
                msg_type = header['type']

                if sid not in senders:
                    senders[sid] = {'name': f'sender_{sid}', 'types': set()}
                senders[sid]['types'].add(msg_type)

                # Parse sender description (contains tracker name)
                if msg_type == -1:  # SENDER_DESCRIPTION
                    text = try_parse_sender_description(payload)
                    if text:
                        senders[sid]['name'] = text
                        print(f'🏷️  [sender {sid}] 发现 Tracker: "{text}"')

                # Parse tracker position (type=1 for tracker service)
                # VRPN tracker service type = 1, position message = (1 << 16) | 1 = 65537
                elif msg_type == 65537 or msg_type == 1:  # tracker position
                    pos = try_parse_tracker_pos(payload)
                    if pos:
                        pos_count += 1
                        tracker_data[sid].append({
                            'pos': pos,
                            'sec': header['tv_sec'],
                            'usec': header['tv_usec'],
                        })

                        name = senders[sid]['name']
                        x, y, z = pos['x'], pos['y'], pos['z']
                        norm = (x**2 + y**2 + z**2) ** 0.5

                        # Determine unit
                        if norm > 500:
                            unit_hint = '🔴 mm级'
                        elif norm > 50:
                            unit_hint = '🟡 cm/dm级'
                        else:
                            unit_hint = '🟢 m级'

                        print(f'📍 [{name}] Pos  |  X={x:+.4f}  Y={y:+.4f}  Z={z:+.4f}  '
                              f'|pos|={norm:.1f}  {unit_hint}')

                # Tracker quaternion can be in different formats
                elif msg_type in (65540, 131076):  # tracker quaternion variants
                    quat = try_parse_tracker_quat(payload)
                    if quat:
                        quat_count += 1
                        # Store with last position for this sender
                        name = senders[sid]['name']
                        print(f'🔄 [{name}] Quat |  '
                              f'qx={quat["qx"]:+.4f}  qy={quat["qy"]:+.4f}  '
                              f'qz={quat["qz"]:+.4f}  qw={quat["qw"]:+.4f}')

                # Also handle raw position type (some servers send tracker pos/vel/acc as different message types)
                elif msg_type & 0xFFFF0000 == 0x00010000:  # tracker service messages
                    subtype = msg_type & 0xFFFF
                    if subtype == 1:  # position
                        pos = try_parse_tracker_pos(payload)
                        if pos:
                            pos_count += 1
                            tracker_data[sid].append({
                                'pos': pos,
                                'sec': header['tv_sec'],
                                'usec': header['tv_usec'],
                            })
                            name = senders[sid]['name']
                            x, y, z = pos['x'], pos['y'], pos['z']
                            norm = (x**2 + y**2 + z**2) ** 0.5
                            unit_hint = '🔴 mm' if norm > 500 else ('🟡 cm/dm' if norm > 50 else '🟢 m')
                            print(f'📍 [{name}] Pos  |  X={x:+.4f}  Y={y:+.4f}  Z={z:+.4f}  '
                                  f'|pos|={norm:.1f}  {unit_hint}')

    except KeyboardInterrupt:
        print('\n\n⏹️  用户中断')
    finally:
        sock.close()
        elapsed = time.time() - start_time

    # ============== Summary ==============
    print('\n' + '=' * 70)
    print('📊 诊断总结')
    print('=' * 70)
    print(f'采集时长: {elapsed:.1f}s')
    print(f'总消息数: {msg_count}')
    print(f'位置消息: {pos_count}')
    print(f'姿态消息: {quat_count}')
    print()

    print(f'发现的 Sender ({len(senders)} 个):')
    for sid, info in sorted(senders.items()):
        print(f'  sender_id={sid}: "{info["name"]}" (types: {[decode_vrpn_type(t) for t in info["types"]]})')

    print()
    for sid, data in sorted(tracker_data.items()):
        if not data:
            continue
        name = senders.get(sid, {}).get('name', f'sender_{sid}')
        xs = [d['pos']['x'] for d in data]
        ys = [d['pos']['y'] for d in data]
        zs = [d['pos']['z'] for d in data]

        norms = [(x**2 + y**2 + z**2)**0.5 for x, y, z in zip(xs, ys, zs)]
        avg_norm = sum(norms) / len(norms)

        print(f'📍 [{name}] 位置数据分析 ({len(data)} 条):')
        print(f'   X: [{min(xs):.4f}, {max(xs):.4f}]  跨度={max(xs)-min(xs):.4f}')
        print(f'   Y: [{min(ys):.4f}, {max(ys):.4f}]  跨度={max(ys)-min(ys):.4f}')
        print(f'   Z: [{min(zs):.4f}, {max(zs):.4f}]  跨度={max(zs)-min(zs):.4f}')
        print(f'   平均 |pos|: {avg_norm:.2f}')

        if avg_norm > 500:
            print(f'   ⚠️  单位判断: 极可能是 **毫米(mm)** → bridge 需 ×0.001')
        elif avg_norm > 50:
            print(f'   ⚠️  单位判断: 可能是 cm/dm → 请根据水池实际尺寸确认')
        else:
            print(f'   ✅ 单位判断: 极可能是 **米(m)** → 可直接使用')

        if len(zs) > 1:
            z_vals = [z for z in zs if abs(z) > 0.01]
            if z_vals:
                z_avg = sum(z_vals) / len(z_vals)
                if z_avg > 0:
                    print(f'   🔍 Z 轴: 均值={z_avg:.2f} (正值) → 可能是 Z-up (需翻转)')
                else:
                    print(f'   🔍 Z 轴: 均值={z_avg:.2f} (负值) → 可能是深度方向 (直接可用)')

    print()
    print('💡 下一步:')
    print('   1. 确认 Tracker 名称 → 更新 nokov_params.yaml 中的 tracker_name')
    print('   2. 确认单位 → 如需要，在 nokov_tf_bridge.py 中添加 scale 转换')
    print('   3. 确认 Z 轴 → 如需要，在 nokov_tf_bridge.py 中添加 Z 翻转')
    print('   4. X/Y 轴 → 推动机器人观察哪个轴变化最大，确认坐标系对应关系')
    print()
    print('🔌 后续 ROS 2 方案:')
    print('   sudo apt install ros-humble-vrpn-mocap')
    print('   ros2 run vrpn_mocap vrpn_mocap_client --ros-args -p server:=10.1.1.198')

    return 0


if __name__ == '__main__':
    sys.exit(main())
