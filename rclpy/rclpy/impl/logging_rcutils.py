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


import importlib

import rclpy.impl.logging_rcutils_config
_rclpy_logging = importlib.import_module('._rclpy_logging', package='rclpy')


def initialize():
    return _rclpy_logging.rclpy_logging_initialize()


def get_severity_threshold():
    return _rclpy_logging.rclpy_logging_get_severity_threshold()


def set_severity_threshold(severity):
    return _rclpy_logging.rclpy_logging_set_severity_threshold(severity)


def log(message, severity, **kwargs):
    # Infer the requested log features from the keyword arguments
    features = rclpy.impl.logging_rcutils_config.get_features_from_kwargs(**kwargs)
    suffix = rclpy.impl.logging_rcutils_config.get_suffix_from_features(features)

    if suffix not in rclpy.impl.logging_rcutils_config.supported_feature_combinations:
        raise AttributeError('invalid combination of logging features: ' + str(features))

    required_params = rclpy.impl.logging_rcutils_config.get_macro_parameters(suffix)

    # Copy only the required arguments into a minimal dictionary.
    # This is required because the C function cannot parse unknown keyword arguments.
    params = {}
    for p, properties in required_params.items():
        scoped_name = properties['scoped_name']
        try:
            params[scoped_name] = kwargs[scoped_name]
        except:
            raise RuntimeError(
                'required parameter "{0}" not specified '
                'but is required for the one of the requested logging features "{1}"'.format(
                    scoped_name, features))

    # Get the relevant function from the C extension
    f = getattr(_rclpy_logging, 'rclpy_logging_log_' + severity.name.lower() + suffix.lower())
    return f(message, **params)
