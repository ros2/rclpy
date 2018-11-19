^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2018-11-19)
------------------
* Updated to use new error handling API from rcutils (`#245 <https://github.com/ros2/rclpy/issues/245>`_)
* Added library path hook for platforms other than Windows. (`#243 <https://github.com/ros2/rclpy/issues/243>`_)
* Avoided use of MethodType when monkey patching for tests (`#239 <https://github.com/ros2/rclpy/issues/239>`_)
* Fixed repeated fini-ing on failure to parse yaml params (`#238 <https://github.com/ros2/rclpy/issues/238>`_)
* Added methods on Mock class for Python 3.5 compatibility (`#237 <https://github.com/ros2/rclpy/issues/237>`_)
* Added getter for tuple with seconds and nanoseconds (`#235 <https://github.com/ros2/rclpy/issues/235>`_)
* Added new method to get node names and namespaces (`#233 <https://github.com/ros2/rclpy/issues/233>`_)
* Fixed warning when parameter value is uninitialized. (`#234 <https://github.com/ros2/rclpy/issues/234>`_)
* Added initial node parameters from a parameters yaml files and constructor arguments. (`#225 <https://github.com/ros2/rclpy/issues/225>`_)
* Added callbacks when time jumps (`#222 <https://github.com/ros2/rclpy/issues/222>`_)
* Updated to use consolidated rcl_wait_set_clear() (`#230 <https://github.com/ros2/rclpy/issues/230>`_)
* Added parameter events publishing (`#226 <https://github.com/ros2/rclpy/issues/226>`_)
* Added Node API method for setting the parameters_callback. (`#228 <https://github.com/ros2/rclpy/issues/228>`_)
* Added test for when sim time is active but unset (`#229 <https://github.com/ros2/rclpy/issues/229>`_)
* Added node parameters and parameter services (`#214 <https://github.com/ros2/rclpy/issues/214>`_)
* Disabled 1kHz test on all platforms (`#223 <https://github.com/ros2/rclpy/issues/223>`_)
* Updated to allow duration to be initialized with negative nanoseconds (`#221 <https://github.com/ros2/rclpy/issues/221>`_)
* Updated to allow Duration to be negative (`#220 <https://github.com/ros2/rclpy/issues/220>`_)
* Added a reference to its executor on Node (`#218 <https://github.com/ros2/rclpy/issues/218>`_)
* Fixed executor.remove_node() (`#217 <https://github.com/ros2/rclpy/issues/217>`_)
* Fixed bool return value for executor.add_node() (`#216 <https://github.com/ros2/rclpy/issues/216>`_)
* Added TimeSource and support for ROS time (`#210 <https://github.com/ros2/rclpy/issues/210>`_)
* Added Time, Duration, Clock wrapping rcl (`#209 <https://github.com/ros2/rclpy/issues/209>`_)
* Contributors: Dirk Thomas, Michael Carroll, Mikael Arguedas, Shane Loretz, Steven! Ragnarök, William Woodall, dhood

0.5.3 (2018-07-17)
------------------
* use test_msgs instead of std_msgs (`#204 <https://github.com/ros2/rclpy/issues/204>`_)
* Fixes memory leaks for nested fields (`#203 <https://github.com/ros2/rclpy/issues/203>`_)
  This separates memory allocation out from convert_from_py function.
  Now it uses separate create_message function to allocate message,
  making it explicit gives better control where and how memory is
  allocated and freed.
* Contributors: Martins Mozeiko, Mikael Arguedas

0.5.1 (2018-06-27)
------------------
* Changed the maintainer to be William Woodall. (`#196 <https://github.com/ros2/rclpy/issues/196>`_)
* Contributors: William Woodall

0.5.0 (2018-06-25)
------------------
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
  * Publish parameter events.
  Adds a parameter event publisher to rclpy nodes.
  * Increase base number of publishers for testing.
  Because every node has a parameter events publisher bump the number of
  expected publishers in a couple of cases.
  * Remove comment now that parameter services are implemented.
  * Delete NOT_SET parameters if present regardless of prior type.
  * Use ParameterMsg rather than RCLParameter for msg type name.
  * Publish parameter events.
  Adds a parameter event publisher to rclpy nodes.
  * Increase base number of publishers for testing.
  Because every node has a parameter events publisher bump the number of
  expected publishers in a couple of cases.
  * Remove comment now that parameter services are implemented.
  * Delete NOT_SET parameters if present regardless of prior type.
  * Use ParameterMsg rather than RCLParameter for msg type name.
  * Publish parameter events.
  Adds a parameter event publisher to rclpy nodes.
  * Increase base number of publishers for testing.
  Because every node has a parameter events publisher bump the number of
  expected publishers in a couple of cases.
  * Remove comment now that parameter services are implemented.
  * Delete NOT_SET parameters if present regardless of prior type.
  * Use ParameterMsg rather than RCLParameter for msg type name.
  * Publish parameter events.
  Adds a parameter event publisher to rclpy nodes.
  * Increase base number of publishers for testing.
  Because every node has a parameter events publisher bump the number of
  expected publishers in a couple of cases.
  * Remove comment now that parameter services are implemented.
  * Delete NOT_SET parameters if present regardless of prior type.
  * Use ParameterMsg rather than RCLParameter for msg type name.
  * Publish parameter events.
  Adds a parameter event publisher to rclpy nodes.
  * Increase base number of publishers for testing.
  Because every node has a parameter events publisher bump the number of
  expected publishers in a couple of cases.
  * Remove comment now that parameter services are implemented.
  * Delete NOT_SET parameters if present regardless of prior type.
  * Use ParameterMsg rather than RCLParameter for msg type name.
  * Publish parameter events.
  Adds a parameter event publisher to rclpy nodes.
  * Increase base number of publishers for testing.
  Because every node has a parameter events publisher bump the number of
  expected publishers in a couple of cases.
  * Remove comment now that parameter services are implemented.
  * Delete NOT_SET parameters if present regardless of prior type.
  * Use ParameterMsg rather than RCLParameter for msg type name.
  * Publish parameter events.
  Adds a parameter event publisher to rclpy nodes.
  * Increase base number of publishers for testing.
  Because every node has a parameter events publisher bump the number of
  expected publishers in a couple of cases.
  * Remove comment now that parameter services are implemented.
  * Delete NOT_SET parameters if present regardless of prior type.
  * Use ParameterMsg rather than RCLParameter for msg type name.
* Contributors: Dirk Thomas, Ethan Gao, Michael Carroll, Mikael Arguedas, Nick Medveditskov, Shane Loretz, Tully Foote, William Woodall, dhood
