"""Exercise worker control flow without ROS, cameras, CUDA or models."""
import ast
from pathlib import Path
from types import SimpleNamespace
import threading


def worker():
    source = Path(__file__).parents[1] / 'auv_competition_control/dual_model_perception_node.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_worker_loop')
    namespace = dict(BALL_STATES={'SEARCH_BALL'}, GATE_POSE_STATES={'ALIGN_GATE'},
                     cv2=SimpleNamespace(putText=lambda *a: None, FONT_HERSHEY_SIMPLEX=0))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 str(source), 'exec'), namespace)
    return namespace['_worker_loop']


def test_no_new_frame_never_republishes_old_image():
    stop = threading.Event()
    class Event:
        def wait(self, timeout):
            stop.set()
            return False
    worker()(SimpleNamespace(_stop_event=stop, _frame_event=Event()))


def test_idle_preview_does_not_run_models():
    stop = threading.Event()
    event = threading.Event()
    event.set()
    class Image:
        def copy(self):
            return self
    published = []
    def publish(image, frame):
        published.append(frame)
        stop.set()
    harness = SimpleNamespace(
        _stop_event=stop, _frame_event=event, _lock=threading.Lock(),
        _publishing_allowed=lambda: True, _latest_image=Image(),
        _latest_image_frame_id='camera', _latest_image_s=10,
        _latest_depth=None, _latest_depth_s=10, _camera_matrix=None,
        _distortion=None, _mission_state='START_DELAY', _now_s=lambda: 10,
        get_parameter=lambda name: SimpleNamespace(value=name == 'publish_debug_image'),
        _publish_debug=publish, _publish_json=lambda *a: None, _diagnostics_pub=None)
    worker()(harness)
    assert published == ['camera']
