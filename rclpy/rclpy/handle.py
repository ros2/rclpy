# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from threading import Lock

from rclpy.impl.implementation_singleton import rclpy_pycapsule_implementation as _rclpy_capsule


class InvalidHandle(Exception):
    pass


class Handle:
    """
    Wrap a pycapsule object for thread-safe early destruction.

    This is intended to be used as a context manager, meaning using the ``with`` keyword.

    ::
    with subscription.handle as pycapsule:
        ...

    This class assumes the passed pycapsule has a destructor.
    When this class destroys the capsule, it will call the destructor.
    Then it will set the destructor to NULL so it is not called a second time when the capsule is
    garbage collected.
    If :meth:`destroy` is never called then the pycapsule will be destructed when it is
    garbage collected.
    """

    def __init__(self, pycapsule):
        self.__capsule = pycapsule
        self.__use_count = 0
        self.__request_invalidation = False
        self.__valid = True
        self.__lock = Lock()
        # Called to give an opportunity to raise an exception if the object is not a pycapsule.
        self.__capsule_name = _rclpy_capsule.rclpy_pycapsule_name(pycapsule)
        self.__capsule_pointer = _rclpy_capsule.rclpy_pycapsule_pointer(pycapsule)

    def __eq__(self, other):
        return self.__capsule_pointer == other.__capsule_pointer

    def __hash__(self):
        return self.__capsule_pointer

    @property
    def name(self):
        """
        Get the name of the managed pycapsule.

        rclpy uses the name of the C type the pycapsule holds a pointer to.

        :return: name of the pycapsule
        """
        return self.__capsule_name

    @property
    def pointer(self):
        """
        Get the address held by the managed pycapsule.

        :return: address of the pycapsule
        """
        return self.__capsule_pointer

    def destroy(self):
        """Destroy pycapsule as soon as possible without waiting for garbage collection."""
        with self.__lock:
            if not self.__valid:
                raise InvalidHandle('Asked to destroy handle, but it was already destroyed')
            self.__request_invalidation = True
            if 0 == self.__use_count:
                self.__destroy()  # calls pycapsule destructor

    def _get_capsule(self):
        """
        Get the pycapsule managed by this handle.

        The capsule must be returned using :meth:`_return_capsule` when it is no longer in use.
        :return: PyCapsule instance
        """
        with self.__lock:
            if not self.__valid:
                raise InvalidHandle('Tried to use a handle that has been destroyed.')
            self.__use_count += 1
        return self.__capsule

    def _return_capsule(self):
        """
        Return the pycapsule that was previously gotten with :meth:`_get_capsule`.

        :return: None
        """
        with self.__lock:
            # Assume _return_capsule is not called more times than _get_capsule
            assert self.__use_count > 0
            self.__use_count -= 1
            if 0 == self.__use_count and self.__request_invalidation:
                self.__destroy()  # calls pycapsule destructor

    def __enter__(self):
        return self._get_capsule()

    def __exit__(self, type_, value, traceback):
        self._return_capsule()

    def __destroy(self):
        # Assume lock is held
        assert not self.__lock.acquire(blocking=False)
        # Assume capsule has not been destroyed
        assert self.__valid
        # Assume no one is using the capsule anymore
        assert self.__use_count == 0
        # Assume someone has asked it to be destroyed
        assert self.__request_invalidation
        _rclpy_capsule.rclpy_pycapsule_destroy(self.__capsule)
        self.__valid = False
