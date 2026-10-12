from sensor_msgs.msg import CameraInfo

from auv_competition_control.zed_sdk_source_node import fill_rectified_camera_info


def test_fill_rectified_camera_info_uses_optical_intrinsics():
    message = CameraInfo()
    fill_rectified_camera_info(message, 1280, 720, 700.0, 701.0, 640.0, 360.0)
    assert message.width == 1280
    assert message.height == 720
    assert list(message.d) == [0.0] * 5
    assert list(message.k) == [700.0, 0.0, 640.0,
                               0.0, 701.0, 360.0,
                               0.0, 0.0, 1.0]
    assert message.p[0] == 700.0
    assert message.p[5] == 701.0


def test_fill_rectified_camera_info_rejects_invalid_calibration():
    try:
        fill_rectified_camera_info(CameraInfo(), 0, 720, 700.0, 701.0, 640.0, 360.0)
    except ValueError as error:
        assert 'invalid rectified camera calibration' in str(error)
    else:
        raise AssertionError('invalid calibration was accepted')
