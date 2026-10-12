import threading

import pytest

from auv_competition_control.dual_model_perception_node import (
    DualModelPerceptionNode,
)


class FakeContext:
    def __init__(self, active=True):
        self.active = active

    def ok(self):
        return self.active


class PublishHarness:
    _publishing_allowed = DualModelPerceptionNode._publishing_allowed
    _publish_json = DualModelPerceptionNode._publish_json

    def __init__(self, active=True):
        self._stop_event = threading.Event()
        self.context = FakeContext(active)


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_json_publish_is_skipped_after_context_shutdown():
    harness = PublishHarness(active=False)
    publisher = CapturePublisher()

    assert harness._publish_json(publisher, {'state': 'SEARCH_BALL'}) is False
    assert publisher.messages == []


def test_context_shutdown_race_does_not_escape_worker():
    harness = PublishHarness(active=True)

    class ShutdownRacePublisher:
        def publish(self, _message):
            harness.context.active = False
            raise RuntimeError('publisher context is invalid')

    assert harness._publish_json(
        ShutdownRacePublisher(), {'state': 'SEARCH_BALL'}) is False


def test_real_publish_error_still_propagates_while_context_is_active():
    harness = PublishHarness(active=True)

    class BrokenPublisher:
        def publish(self, _message):
            raise RuntimeError('real transport failure')

    with pytest.raises(RuntimeError, match='real transport failure'):
        harness._publish_json(BrokenPublisher(), {'state': 'SEARCH_BALL'})
