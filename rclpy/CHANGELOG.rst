^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Changed the rclpy signal handler so that it is registered in ``rclpy_init()`` rather than in each wait. (`#194 <https://github.com/ros2/rclpy/issues/194>`_)
* Changed the signal handler in rclpy to call the original signal handler when receiving SIGINT during a wait on a wait set. (`#191 <https://github.com/ros2/rclpy/issues/191>`_)
* Added API for counting the number of publishers and subscribers on a topic. (`#183 <https://github.com/ros2/rclpy/issues/183>`_)
* Updated Node interface so it can use the command line arguments and can optionally ignore global arguments. (`#185 <https://github.com/ros2/rclpy/issues/185>`_)
* Changed the ``rclpy.spin*()`` functions to use a persistent executor. (`#176 <https://github.com/ros2/rclpy/issues/176>`_)
* Fixed a bug related to zero-initialization. (`#182 <https://github.com/ros2/rclpy/issues/182>`_)
* Added code to handle node names which are ``nullptr``. (`#177 <https://github.com/ros2/rclpy/issues/177>`_)
* Refactored client class so that it can handle multiple requests. (`#170 <https://github.com/ros2/rclpy/issues/170>`_)
* Fixed ``rclpy_init()`` so that it actually passes command line arguments to ``rcl_init()`` (`#179 <https://github.com/ros2/rclpy/issues/179>`_)
* Changed logging to get the node's logger name from rcl. (`#174 <https://github.com/ros2/rclpy/issues/174>`_)
* Fixed a bug where ``rclpy_take_response()`` was ignoring the sequence number. (`#171 <https://github.com/ros2/rclpy/issues/171>`_)
* Added support for Futures and coroutines in the executor. (`#166 <https://github.com/ros2/rclpy/issues/166>`_)
* Updated code to match API change needed to avoid accidental nullptr dereference. (`#157 <https://github.com/ros2/rclpy/issues/157>`_)
  * Signed-off-by: Ethan Gao <ethan.gao@linux.intel.com>
* Added a sleep to workaround race condition in MultiThreadedExecutor test. (`#168 <https://github.com/ros2/rclpy/issues/168>`_)
* Disable 1kHz timer tests on the ARM architectures. (`#169 <https://github.com/ros2/rclpy/issues/169>`_)
* Contributors: Dirk Thomas, Ethan Gao, Michael Carroll, Mikael Arguedas, Nick Medveditskov, Shane Loretz, Tully Foote, William Woodall, dhood
