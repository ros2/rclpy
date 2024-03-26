# generated from rosidl_generator_py/resource/_idl.py.em
# with input from action_msgs:srv/CancelGoal.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CancelGoal_Request(type):
    """Metaclass of message 'CancelGoal_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('action_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'action_msgs.srv.CancelGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__cancel_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__cancel_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__cancel_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__cancel_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__cancel_goal__request

            from action_msgs.msg import GoalInfo
            if GoalInfo.__class__._TYPE_SUPPORT is None:
                GoalInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CancelGoal_Request(metaclass=Metaclass_CancelGoal_Request):
    """Message class 'CancelGoal_Request'."""

    __slots__ = [
        '_goal_info',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'goal_info': 'action_msgs/GoalInfo',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['action_msgs', 'msg'], 'GoalInfo'),  # noqa: E501
    )

    def __init__(self, **kwargs) -> None:
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from action_msgs.msg import GoalInfo
        self.goal_info = kwargs.get('goal_info', GoalInfo())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_info != other.goal_info:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_info(self):
        """Message field 'goal_info'."""
        return self._goal_info

    @goal_info.setter
    def goal_info(self, value):
        if self._check_fields:
            from action_msgs.msg import GoalInfo
            assert \
                isinstance(value, GoalInfo), \
                "The 'goal_info' field must be a sub message of type 'GoalInfo'"
        self._goal_info = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CancelGoal_Response(type):
    """Metaclass of message 'CancelGoal_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ERROR_NONE': 0,
        'ERROR_REJECTED': 1,
        'ERROR_UNKNOWN_GOAL_ID': 2,
        'ERROR_GOAL_TERMINATED': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('action_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'action_msgs.srv.CancelGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__cancel_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__cancel_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__cancel_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__cancel_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__cancel_goal__response

            from action_msgs.msg import GoalInfo
            if GoalInfo.__class__._TYPE_SUPPORT is None:
                GoalInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ERROR_NONE': cls.__constants['ERROR_NONE'],
            'ERROR_REJECTED': cls.__constants['ERROR_REJECTED'],
            'ERROR_UNKNOWN_GOAL_ID': cls.__constants['ERROR_UNKNOWN_GOAL_ID'],
            'ERROR_GOAL_TERMINATED': cls.__constants['ERROR_GOAL_TERMINATED'],
        }

    @property
    def ERROR_NONE(self):
        """Message constant 'ERROR_NONE'."""
        return Metaclass_CancelGoal_Response.__constants['ERROR_NONE']

    @property
    def ERROR_REJECTED(self):
        """Message constant 'ERROR_REJECTED'."""
        return Metaclass_CancelGoal_Response.__constants['ERROR_REJECTED']

    @property
    def ERROR_UNKNOWN_GOAL_ID(self):
        """Message constant 'ERROR_UNKNOWN_GOAL_ID'."""
        return Metaclass_CancelGoal_Response.__constants['ERROR_UNKNOWN_GOAL_ID']

    @property
    def ERROR_GOAL_TERMINATED(self):
        """Message constant 'ERROR_GOAL_TERMINATED'."""
        return Metaclass_CancelGoal_Response.__constants['ERROR_GOAL_TERMINATED']


class CancelGoal_Response(metaclass=Metaclass_CancelGoal_Response):
    """
    Message class 'CancelGoal_Response'.

    Constants:
      ERROR_NONE
      ERROR_REJECTED
      ERROR_UNKNOWN_GOAL_ID
      ERROR_GOAL_TERMINATED
    """

    __slots__ = [
        '_return_code',
        '_goals_canceling',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'return_code': 'int8',
        'goals_canceling': 'sequence<action_msgs/GoalInfo>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['action_msgs', 'msg'], 'GoalInfo')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.return_code = kwargs.get('return_code', int())
        self.goals_canceling = kwargs.get('goals_canceling', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.return_code != other.return_code:
            return False
        if self.goals_canceling != other.goals_canceling:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def return_code(self):
        """Message field 'return_code'."""
        return self._return_code

    @return_code.setter
    def return_code(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'return_code' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'return_code' field must be an integer in [-128, 127]"
        self._return_code = value

    @builtins.property
    def goals_canceling(self):
        """Message field 'goals_canceling'."""
        return self._goals_canceling

    @goals_canceling.setter
    def goals_canceling(self, value):
        if self._check_fields:
            from action_msgs.msg import GoalInfo
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, GoalInfo) for v in value) and
                 True), \
                "The 'goals_canceling' field must be a set or sequence and each value of type 'GoalInfo'"
        self._goals_canceling = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CancelGoal_Event(type):
    """Metaclass of message 'CancelGoal_Event'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('action_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'action_msgs.srv.CancelGoal_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__cancel_goal__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__cancel_goal__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__cancel_goal__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__cancel_goal__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__cancel_goal__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CancelGoal_Event(metaclass=Metaclass_CancelGoal_Event):
    """Message class 'CancelGoal_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<action_msgs/CancelGoal_Request, 1>',
        'response': 'sequence<action_msgs/CancelGoal_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['action_msgs', 'srv'], 'CancelGoal_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['action_msgs', 'srv'], 'CancelGoal_Response'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from action_msgs.srv import CancelGoal_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, CancelGoal_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'CancelGoal_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from action_msgs.srv import CancelGoal_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, CancelGoal_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'CancelGoal_Response'"
        self._response = value


class Metaclass_CancelGoal(type):
    """Metaclass of service 'CancelGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('action_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'action_msgs.srv.CancelGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__cancel_goal

            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal_Request._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal_Request.__import_type_support__()
            if _cancel_goal.Metaclass_CancelGoal_Response._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal_Response.__import_type_support__()
            if _cancel_goal.Metaclass_CancelGoal_Event._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal_Event.__import_type_support__()


from typing import Type, ClassVar, Final, NoReturn

from typing_extensions import Never


class CancelGoal(metaclass=Metaclass_CancelGoal):
    # from action_msgs.srv._cancel_goal import CancelGoal_Request
    # from action_msgs.srv._cancel_goal import CancelGoal_Response
    # from action_msgs.srv._cancel_goal import CancelGoal_Event

    # Self: Type[Metaclass_CancelGoal]] = Metaclass_CancelGoal
    # Request: ClassVar[Type[CancelGoal_Request]] = CancelGoal_Request
    # Response: ClassVar[Type[CancelGoal_Response]] = CancelGoal_Response
    # Event: ClassVar[Type[CancelGoal_Event]] = CancelGoal_Event

    Request: Type[CancelGoal_Request] = CancelGoal_Request
    Response: Type[CancelGoal_Response] = CancelGoal_Response
    Event: Type[CancelGoal_Event] = CancelGoal_Event

    def __init__(self) -> NoReturn:
        raise NotImplementedError('Service classes can not be instantiated')
