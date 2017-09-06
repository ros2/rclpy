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


from collections import OrderedDict
try:
    import cPickle as pickle
except ImportError:
    import pickle
import importlib
import inspect
import time

_rclpy_logging = importlib.import_module('._rclpy_logging', package='rclpy')

# Known filenames from which logging methods can be called (will be ignored in `_find_caller`)
_known_callers = []
_internal_callers = None  # This will be populated on first logging call.


def _find_caller(frame):
    """Get the first calling frame that is outside of rclpy."""
    global _internal_callers
    if _internal_callers is None:
        # Populate the list of internal filenames from which logging methods can be called.
        # This has to be done from within a function to avoid cyclic module imports.
        import rclpy.logging
        _internal_callers = [__file__, rclpy.logging.__file__]
        # Preserve any filenames that may have been added by third parties.
        _known_callers.extend(_internal_callers)

    frame_filename = inspect.getabsfile(frame)
    while any(f in frame_filename for f in _known_callers):
        frame = frame.f_back
        frame_filename = inspect.getabsfile(frame)
    return frame


class CallerId:

    def __init__(self, frame=None):
        if not frame:
            frame = _find_caller(inspect.currentframe())
        self.function_name = frame.f_code.co_name
        self.file_name = inspect.getabsfile(frame)
        self.line_number = frame.f_lineno
        self.last_index = frame.f_lasti  # To distinguish between two callers on the same line


class LoggingFilter:
    """Base class for logging filters."""

    """
    Parameters of a filter and their default value, if appropriate.

    A default value of None makes a parameter required.
    """
    params = {}

    """
    Initialize the context of a logging call, e.g. declare variables needed for
    determining the log condition and add them to the context.
    """
    @classmethod
    def initialize_context(cls, context, **kwargs):
        # Store all parameters in the context so we can check that users never try to change them.
        for param in cls.params:
            context[param] = kwargs.get(param, cls.params[param])
            if context[param] is None:
                raise TypeError(
                    'Required parameter "{0}" was not specified for logging filter "{1}"'
                    .format(param, cls.__name__))

    """
    Decide if it's appropriate to log given a context, and update the context accordingly.
    """
    @staticmethod
    def should_log(context):
        return True


class Once(LoggingFilter):
    """Ignore all log calls except the first one."""

    params = {
        'once': None,
    }

    @classmethod
    def initialize_context(cls, context, **kwargs):
        super(Once, cls).initialize_context(context, **kwargs)
        context['has_been_logged_once'] = False

    @staticmethod
    def should_log(context):
        logging_condition = False
        if not context['has_been_logged_once']:
            logging_condition = True
            context['has_been_logged_once'] = True
        return logging_condition


class Throttle(LoggingFilter):
    """Ignore log calls if the last call is not longer ago than the specified duration."""

    params = {
        'throttle_duration_sec': None,
        'throttle_time_source_type': 'RCUTILS_STEADY_TIME',
    }

    @classmethod
    def initialize_context(cls, context, **kwargs):
        super(Throttle, cls).initialize_context(context, **kwargs)
        context['throttle_last_logged'] = 0

    @staticmethod
    def should_log(context):
        logging_condition = True
        # TODO(dhood): use rcutils time and the time source type
        now = time.time()
        next_log_time = context['throttle_last_logged'] + context['throttle_duration_sec']
        logging_condition = now >= next_log_time
        if logging_condition:
            context['throttle_last_logged'] = now
        return logging_condition


class SkipFirst(LoggingFilter):
    """Ignore the first log call but process all subsequent calls."""

    params = {
        'skip_first': None,
    }

    @classmethod
    def initialize_context(cls, context, **kwargs):
        super(SkipFirst, cls).initialize_context(context, **kwargs)
        context['first_has_been_skipped'] = False

    @staticmethod
    def should_log(context):
        logging_condition = True
        if not context['first_has_been_skipped']:
            logging_condition = False
            context['first_has_been_skipped'] = True
        return logging_condition


# The ordering of this dictionary defines the order in which filters will be processed.
supported_filters = OrderedDict()
supported_filters['throttle'] = Throttle
supported_filters['skip_first'] = SkipFirst
supported_filters['once'] = Once


def get_filters_from_kwargs(**kwargs):
    """
    Determine which filters have had parameters specified in the given keyword arguments.

    Returns the list of filters using the order specified by `supported_filters`.
    """
    detected_filters = []
    all_supported_params = []
    for supported_filter, filter_class in supported_filters.items():
        filter_params = filter_class.params.keys()
        all_supported_params.extend(filter_params)
        if any(kwargs.get(param_name) for param_name in filter_params):
            detected_filters.append(supported_filter)
    # Check that all required parameters (with no default value) have been specified
    for detected_filter in detected_filters:
        for param_name, default_value in supported_filters[detected_filter].params.items():
            if param_name not in kwargs:
                if default_value is not None:
                    kwargs[param_name] = default_value
                else:
                    raise TypeError(
                        'required parameter "{0}" not specified '
                        'but is required for the the logging filter "{1}"'.format(
                            param_name, detected_filter))
    for kwarg in kwargs:
        if kwarg not in all_supported_params:
            raise TypeError(
                'parameter "{0}" is not one of the recognized logging options "{1}"'
                .format(kwarg, all_supported_params)
            )
    return detected_filters


class RcutilsLogger:

    def __init__(self, name=''):
        self.name = name
        self.contexts = {}

    def get_severity_threshold(self):
        return _rclpy_logging.rclpy_logging_get_severity_threshold()

    def set_severity_threshold(self, severity):
        return _rclpy_logging.rclpy_logging_set_severity_threshold(severity)

    def log(self, message, severity, **kwargs):
        name = kwargs.pop('name', self.name)

        # Infer the requested log filters from the keyword arguments
        detected_filters = get_filters_from_kwargs(**kwargs)

        # Get/prepare the context corresponding to the caller.
        caller_id = CallerId()
        caller_id_str = pickle.dumps(caller_id)
        if caller_id_str not in self.contexts:
            context = {'name': name, 'severity': severity}
            for detected_filter in detected_filters:
                if detected_filter in supported_filters:
                    supported_filters[detected_filter].initialize_context(context, **kwargs)
            context['filters'] = detected_filters
            self.contexts[caller_id_str] = context
        else:
            context = self.contexts[caller_id_str]
            # Don't support any changes to the logger.
            if severity != context['severity']:
                raise ValueError('Logger severity cannot be changed between calls.')
            if name != context['name']:
                raise ValueError('Logger name cannot be changed between calls.')
            if detected_filters != context['filters']:
                raise ValueError('Requested logging filters cannot be changed between calls.')
            for detected_filter in detected_filters:
                filter_params = supported_filters[detected_filter].params
                if any(context[p] != kwargs.get(p, filter_params[p]) for p in filter_params):
                    raise ValueError(
                        'Logging filter parameters cannot be changed between calls.')

        # Determine if it's appropriate to process the message (any filter can vote no)
        # Note(dhood): even if a message doesn't get logged, a filter might still update its state
        # as if it had been. This matches the behavior of the C logging macros provided by rcutils.
        make_log_call = True
        for logging_filter in context['filters']:
            make_log_call &= supported_filters[logging_filter].should_log(
                self.contexts[caller_id_str])
            if not make_log_call:
                break
        # Only log the message if the severity is appropriate.
        make_log_call &= severity >= self.get_severity_threshold()
        if make_log_call:
            # Get and call the relevant function from the C extension.
            log_function = getattr(_rclpy_logging, 'rclpy_logging_rcutils_log')
            log_function(
                severity, name, message,
                caller_id.function_name, caller_id.file_name, caller_id.line_number)
        return make_log_call

    def debug(self, message, **kwargs):
        from rclpy.logging import LoggingSeverity
        return self.log(message, LoggingSeverity.DEBUG, **kwargs)

    def info(self, message, **kwargs):
        from rclpy.logging import LoggingSeverity
        return self.log(message, LoggingSeverity.INFO, **kwargs)

    def warn(self, message, **kwargs):
        from rclpy.logging import LoggingSeverity
        return self.log(message, LoggingSeverity.WARN, **kwargs)

    def error(self, message, **kwargs):
        from rclpy.logging import LoggingSeverity
        return self.log(message, LoggingSeverity.ERROR, **kwargs)

    def fatal(self, message, **kwargs):
        from rclpy.logging import LoggingSeverity
        return self.log(message, LoggingSeverity.FATAL, **kwargs)
