from hbba_lite.filter_states import OnOffHbbaFilterState, ThrottlingHbbaFilterState


class _HbbaPublisher:
    def __init__(self, node, data_class, topic_name, filter_state_class, state_service_name, qos_profile):
        if state_service_name is None:
            state_service_name = topic_name + '/filter_state'

        self._filter_state = filter_state_class(node, state_service_name)
        self._publisher = node.create_publisher(data_class, topic_name, qos_profile)

    def publish(self, msg):
        if self._filter_state.check():
            self._publisher.publish(msg)

    @property
    def is_filtering_all_messages(self):
        return self._filter_state._is_filtering_all_messages

    def on_filter_state_changing(self, callback):
        def publish_forced(msg):
            self._publisher.publish(msg)

        def filter_callback(*args):
            callback(publish_forced, *args)

        self._filter_state.on_changing(filter_callback)

    def on_filter_state_changed(self, callback):
        self._filter_state.on_changed(callback)


class OnOffHbbaPublisher(_HbbaPublisher):
    def __init__(self, node, data_class, topic_name, qos_profile, state_service_name=None):
        super(OnOffHbbaPublisher, self).__init__(node,
                                                 data_class, topic_name, OnOffHbbaFilterState,
                                                 state_service_name, qos_profile)


class ThrottlingHbbaPublisher(_HbbaPublisher):
    def __init__(self, node, data_class, topic_name, qos_profile, state_service_name=None):
        super(ThrottlingHbbaPublisher, self).__init__(node,
                                                      data_class, topic_name, ThrottlingHbbaFilterState,
                                                      state_service_name, qos_profile)
