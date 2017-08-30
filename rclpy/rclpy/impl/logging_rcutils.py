# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


try:
    import cPickle as pickle
except ImportError:
    import pickle
import importlib
import inspect
import time

_rclpy_logging = importlib.import_module('._rclpy_logging', package='rclpy')
_rclpy_logging.rclpy_logging_initialize()


def _frame_to_caller_id(frame):
    caller_id = (
        inspect.getabsfile(frame),
        frame.f_lineno,
        frame.f_lasti,
    )
    return pickle.dumps(caller_id)


class LoggingFeature:
    """Base class for logging features."""

    """
    Parameters of a feature and their default value, if appropriate.

    A default value of None makes a parameter required.
    """
    params = {}

    """
    Initialize the context of a logging call, e.g. declare variables needed for

    determining the log condition and add them to the context.
    """
    @staticmethod
    def initialize_context(context, **kwargs):
        pass

    """
    Decide if it's appropriate to log given a context, and update the context accordingly.
    """
    @staticmethod
    def log_condition(context):
        return True


class Once(LoggingFeature):
    params = {
        'once': None,
    }

    @staticmethod
    def initialize_context(context, **kwargs):
        context['has_been_logged_once'] = False

    @staticmethod
    def log_condition(context):
        logging_condition = False
        if not context['has_been_logged_once']:
            logging_condition = True
            context['has_been_logged_once'] = True
        return logging_condition


class Throttle(LoggingFeature):
    params = {
        'throttle_duration': None,
        'throttle_time_source_type': 'RCUTILS_STEADY_TIME',
    }

    @staticmethod
    def initialize_context(context, **kwargs):
        context['throttle_duration'] = kwargs['throttle_duration']
        context['throttle_last_logged'] = 0

    @staticmethod
    def log_condition(context):
        logging_condition = True
        # TODO(dhood): use rcutils time and the time source type
        now = time.time()
        logging_condition = now >= context['throttle_last_logged'] + context['throttle_duration']
        if logging_condition:
            context['throttle_last_logged'] = now
        return logging_condition


class SkipFirst(LoggingFeature):
    params = {
        'skip_first': None,
    }

    @staticmethod
    def initialize_context(context, **kwargs):
        context['first_has_been_skipped'] = False

    @staticmethod
    def log_condition(context):
        logging_condition = True
        if not context['first_has_been_skipped']:
            logging_condition = False
            context['first_has_been_skipped'] = True
        return logging_condition


supported_features = {
    'throttle': Throttle,
    'once': Once,
    'skip_first': SkipFirst,
}


def get_features_from_kwargs(**kwargs):
    detected_features = []
    for feature, feature_class in supported_features.items():
        if any(kwargs.get(param_name) for param_name in feature_class.params.keys()):
            detected_features.append(feature)
    # Check that all required parameters (with no default value) have been specified
    for feature in detected_features:
        for param_name, default_value in supported_features[feature].params.items():
            if param_name not in kwargs:
                if default_value is not None:
                    kwargs[param_name] = default_value
                else:
                    raise RuntimeError(
                        'required parameter "{0}" not specified '
                        'but is required for the the logging feature "{1}"'.format(
                            param_name, feature))
    # TODO(dhood): warning for unused kwargs?
    return detected_features


class RcutilsLogger:

    _contexts = {}

    def __init__(self, name=''):
        self.name = name

    def get_severity_threshold(self):
        return _rclpy_logging.rclpy_logging_get_severity_threshold()

    def set_severity_threshold(self, severity):
        return _rclpy_logging.rclpy_logging_set_severity_threshold(severity)

    def log(self, message, severity, **kwargs):
        # Infer the requested log features from the keyword arguments
        features = get_features_from_kwargs(**kwargs)

        name = kwargs.get('name', self.name)
        caller_id = kwargs.get(
            'caller_id',
            _frame_to_caller_id(inspect.currentframe().f_back.f_back))
        if caller_id not in self._contexts:
            context = {'name': name, 'severity': severity}
            for feature in features:
                if feature in supported_features:
                    supported_features[feature].initialize_context(context, **kwargs)
            self._contexts[caller_id] = context

        make_log_call = True
        for feature in features:
            if feature in supported_features:
                make_log_call &= supported_features[feature].log_condition(
                    self._contexts[caller_id])
        if make_log_call:
            # Get the relevant function from the C extension
            f = getattr(_rclpy_logging, 'rclpy_logging_log_' + severity.name.lower())
            f(name, message)
