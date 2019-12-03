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

from threading import RLock
import weakref

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
        self.__rlock = RLock()
        # Create this early because RLock() can raise during interpeter shutdown
        self.__dependents_rlock = RLock()
        self.__required_handles = []
        self.__dependent_handles = weakref.WeakSet()
        self.__destroy_callbacks = []
        # Called to give an opportunity to raise an exception if the object is not a pycapsule.
        self.__capsule_name = _rclpy_capsule.rclpy_pycapsule_name(pycapsule)
        self.__capsule_pointer = _rclpy_capsule.rclpy_pycapsule_pointer(pycapsule)

    def __bool__(self):
        """Return True if the handle is valid."""
        return self.__valid

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

    def destroy(self, then=None):
        """
        Destroy pycapsule as soon as possible without waiting for garbage collection.

        :param then: callback to call after handle has been destroyed.
        """
        with self.__rlock:
            if not self.__valid:
                raise InvalidHandle('Asked to destroy handle, but it was already destroyed')
            if then:
                self.__destroy_callbacks.append(then)
            self.__request_invalidation = True
            if 0 == self.__use_count:
                self.__destroy()

    def requires(self, req_handle):
        """
        Indicate that this handle requires another handle to live longer than itself.

        Calling :meth:`destroy` on the passed in handle will cause this handle to be
        destroyed first.
        This handle will hold a reference to the passed in handle so the required handle is
        garbage collected after this handle in case :meth:`destroy` is not called.
        """
        assert isinstance(req_handle, Handle)
        with self.__rlock, req_handle.__rlock:
            if not self.__valid:
                raise InvalidHandle('Cannot require a new handle if already destroyed')
            if req_handle.__valid:
                self.__required_handles.append(req_handle)
                req_handle.__dependent_handles.add(self)
            else:
                # required handle destroyed before we could link to it, destroy self
                self.destroy()

    def _get_capsule(self):
        """
        Get the pycapsule managed by this handle.

        The capsule must be returned using :meth:`_return_capsule` when it is no longer in use.
        :return: PyCapsule instance
        """
        with self.__rlock:
            if not self.__valid:
                raise InvalidHandle('Tried to use a handle that has been destroyed.')
            self.__use_count += 1
        return self.__capsule

    def _return_capsule(self):
        """
        Return the pycapsule that was previously gotten with :meth:`_get_capsule`.

        :return: None
        """
        with self.__rlock:
            # Assume _return_capsule is not called more times than _get_capsule
            assert self.__use_count > 0
            self.__use_count -= 1
            if 0 == self.__use_count and self.__request_invalidation:
                self.__destroy()

    def __enter__(self):
        return self._get_capsule()

    def __exit__(self, type_, value, traceback):
        self._return_capsule()

    def __destroy(self):
        # Assume no one is using the capsule anymore
        assert self.__use_count == 0
        # Assume someone has asked it to be destroyed
        assert self.__request_invalidation
        # mark as invalid so no one else tries to use it
        self.__valid = False
        self.__destroy_dependents(then=self.__destroy_self)

    def __destroy_dependents(self, then):
        # assumes self.__rlock is held
        deps_lock = self.__dependents_rlock
        # Turn weak references to regular references
        dependent_handles = list(self.__dependent_handles)

        if not dependent_handles:
            # no dependents to wait on
            then()
            return

        def remove_dependent(handle):
            nonlocal dependent_handles
            nonlocal deps_lock
            nonlocal then
            with deps_lock:
                dependent_handles.remove(handle)
                if 0 == len(dependent_handles):
                    # all dependents destroyed, do what comes next
                    then()

        for dep in self.__dependent_handles:
            try:
                dep.destroy(then=remove_dependent)
            except InvalidHandle:
                # Dependent was already destroyed
                remove_dependent(dep)

    def __destroy_self(self):
        with self.__rlock:
            # Calls pycapsule destructor
            _rclpy_capsule.rclpy_pycapsule_destroy(self.__capsule)
            # Call post-destroy callbacks
            while self.__destroy_callbacks:
                self.__destroy_callbacks.pop()(self)
            # get rid of references to other handles to break reference cycles
            del self.__required_handles

    def __del__(self):
        print('destroying', self.__capsule_name, ' ', self.__capsule_pointer)
