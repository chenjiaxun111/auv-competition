import threading
import time
import urllib.request
import urllib.error
import pytest
from http.server import ThreadingHTTPServer
from auv_competition_control.perception_web_viewer import LatestFrame, handler_for


def test_latest_frame_freshness_and_rejection():
    store = LatestFrame()
    assert not store.snapshot(1)[1]['fresh']
    store.put(b'not jpeg', 1)
    assert store.snapshot(1)[1]['frames'] == 0
    store.put(b'\xff\xd8test\xff\xd9', 2)
    assert store.snapshot(3)[0]
    assert not store.snapshot(4.01)[0]
    assert not store.snapshot(1)[0]


def test_http_headers_and_stale_image():
    store = LatestFrame()
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(store))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        with urllib.request.urlopen(base) as r:
            assert 'no-store' == r.headers['Cache-Control']
            assert b'YOLO' in r.read()
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + '/image.jpg')
        assert e.value.code == 503
        store.put(b'\xff\xd8test\xff\xd9', time.monotonic())
        with urllib.request.urlopen(base + '/image.jpg') as r:
            assert r.headers['Content-Type'] == 'image/jpeg'
            assert r.read() == b'\xff\xd8test\xff\xd9'
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
