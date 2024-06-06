from abc import ABC, abstractmethod

from hbba_lite_srvs.srv import SetOnOffFilterState, SetThrottlingFilterState


class _HbbaFilterState(ABC):
    @abstractmethod
    def check(self):
        return False

    @abstractmethod
    def on_changed(self, callback):
        pass


class OnOffHbbaFilterState(_HbbaFilterState):
    def __init__(self, node, state_service_name):
        self._state_service = node.create_service(SetOnOffFilterState, state_service_name, self._state_service_callback)
        self._is_filtering_all_messages = True
        self._on_changing_callback = None
        self._on_changed_callback = None

    def _state_service_callback(self, request, response):
        previous_is_filtering_all_messages = self._is_filtering_all_messages
        if self._on_changing_callback is not None:
            self._on_changing_callback(previous_is_filtering_all_messages, request.is_filtering_all_messages)

        self._is_filtering_all_messages = request.is_filtering_all_messages

        if self._on_changed_callback is not None:
            self._on_changed_callback(previous_is_filtering_all_messages, self._is_filtering_all_messages)

        response.ok = True
        return response

    def on_changing(self, callback):
        self._on_changing_callback = callback

    def on_changed(self, callback):
        self._on_changed_callback = callback

    def check(self):
        return not self._is_filtering_all_messages

    @property
    def is_filtering_all_messages(self):
        return self._is_filtering_all_messages


class ThrottlingHbbaFilterState(_HbbaFilterState):
    def __init__(self, node, state_service_name):
        self._state_service = node.create_service(SetThrottlingFilterState, state_service_name, self._state_service_callback)
        self._is_filtering_all_messages = True
        self._rate = 1
        self._counter = 0
        self._on_changing_callback = None
        self._on_changed_callback = None

    def _state_service_callback(self, request, response):
        if request.rate <= 0:
            response.ok = False
            return response

        previous_is_filtering_all_messages = self._is_filtering_all_messages
        previous_rate = self._rate
        if self._on_changing_callback is not None:
            self._on_changing_callback(previous_is_filtering_all_messages, request.is_filtering_all_messages,
                                       previous_rate, request.rate)

        self._is_filtering_all_messages = request.is_filtering_all_messages
        self._rate = request.rate
        self._counter = 0
        if self._on_changed_callback is not None:
            self._on_changed_callback(previous_is_filtering_all_messages, self._is_filtering_all_messages,
                                      previous_rate, self._rate)

        response.ok = True
        return response

    def on_changing(self, callback):
        self._on_changing_callback = callback

    def on_changed(self, callback):
        self._on_changed_callback = callback

    def check(self):
        if self._is_filtering_all_messages:
            return False

        is_ready = False
        if self._counter == 0:
            is_ready = True
        self._counter = (self._counter + 1) % self._rate

        return is_ready

    @property
    def is_filtering_all_messages(self):
        return self._is_filtering_all_messages
