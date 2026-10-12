"""Read-only JPEG browser preview. No camera ownership, models, RPC or commands."""
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class LatestFrame:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = b''
        self.received = -float('inf')
        self.frames = 0

    def put(self, data, now):
        # Reject malformed/oversized payloads; keep exactly one latest JPEG.
        data = bytes(data)
        if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9') or len(data) > 8_000_000:
            return
        with self.lock:
            self.data, self.received = data, now
            self.frames += 1

    def snapshot(self, now):
        with self.lock:
            age = now - self.received
            fresh = bool(self.data) and 0 <= age <= 2
            return (self.data if fresh else b''), dict(
                fresh=fresh, age_s=round(age, 3) if self.frames else None,
                frames=self.frames)


PAGE = '''<!doctype html><html lang="zh"><meta charset="utf-8">
<title>AUV 实时检测</title><style>body{background:#151b22;color:white;font:18px sans-serif}
img{max-width:100%;max-height:85vh}#status{padding:12px}</style>
<h2>AUV YOLO / 门框 Pose 实时图像</h2>
<p>只读预览，不控制机器人。等待/直行阶段只显示相机图像；无框不一定是漏检。</p>
<div id="status">等待图像…</div><img id="image" hidden>
<script>async function poll(){try{const r=await fetch('/image.jpg?t='+Date.now(),{cache:'no-store'});
if(!r.ok)throw Error('图像未收到或超过2秒未更新');const b=await r.blob();
const u=URL.createObjectURL(b);const im=document.getElementById('image');
im.onload=()=>URL.revokeObjectURL(u);im.src=u;im.hidden=false;
document.getElementById('status').textContent='实时图像（预览最多5 FPS，不是推理帧率）';
}catch(e){document.getElementById('image').hidden=true;
document.getElementById('status').textContent='等待/断流：'+e.message;}
setTimeout(poll,200);}poll();</script></html>'''.encode('utf-8')


def handler_for(store):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            frame, status = store.snapshot(time.monotonic())
            if path == '/':
                code, content, body = 200, 'text/html; charset=utf-8', PAGE
            elif path == '/image.jpg':
                code, content, body = (200 if frame else 503), 'image/jpeg', frame
            elif path == '/status.json':
                code, content, body = 200, 'application/json', json.dumps(status, allow_nan=False).encode()
            else:
                code, content, body = 404, 'text/plain', b'Not found'
            try:
                self.send_response(code)
                self.send_header('Content-Type', content)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def setup(self):
            super().setup()
            self.connection.settimeout(3)

        def log_message(self, *_):
            pass
    return Handler


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = Node('perception_web_viewer')
    node.declare_parameter('host', '0.0.0.0')
    node.declare_parameter('port', 8081)
    node.declare_parameter('image_topic', '/auv/perception/debug/image/compressed')
    port = int(node.get_parameter('port').value)
    if not 1 <= port <= 65535:
        raise ValueError('port must be 1..65535')
    store = LatestFrame()
    server = None
    thread = None
    try:
        sub = node.create_subscription(
            CompressedImage, str(node.get_parameter('image_topic').value),
            lambda msg: store.put(msg.data, time.monotonic()), qos_profile_sensor_data)
        server = ThreadingHTTPServer((str(node.get_parameter('host').value), port), handler_for(store))
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        node.get_logger().info(f'Read-only preview http://JETSON_IP:{port} (trusted LAN only)')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal SIGINT and launch's forwarded SIGINT can arrive separately.
        # Once shutdown has begun, allow HTTP cleanup to complete uninterrupted.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
