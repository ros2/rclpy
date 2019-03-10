^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.3 (2019-03-09)
------------------
* Backport Python Actions (`#282 <https://github.com/ros2/rclpy/issues/282>`_)
  * Add Action Client (`#262 <https://github.com/ros2/rclpy/issues/262>`_)
  * Add rclpy_action module
  * Implement action client
  * Move common conversion function and typedefs to shared header file (impl/common.h)
  * Add tests using mock action server
  * Add action module for aggregating action related submodules
  * Extend Waitable API so executors are aware of Futures
  * Move check_for_type_support() to its own module
  * Fix Executor not executing tasks if there are no ready entities in the wait set (`#272 <https://github.com/ros2/rclpy/issues/272>`_)
  * Fix Node's reference to executor (`#275 <https://github.com/ros2/rclpy/issues/275>`_)
  * Abstract type conversions into functions (`#269 <https://github.com/ros2/rclpy/issues/269>`_)
  * Abstract type conversions into functions
  * Move common C functions to a shared library 'rclpy_common'
  * Add ActionServer (`#270 <https://github.com/ros2/rclpy/issues/270>`_)
  * Add Action server functions to extension module
  * Separated service related macros into separate request and response calls
  * Add server goal handle functions to extension module
  * Update Action extension module to use conversion functions
  * Add implementation of Python ActionServer
  * Handles goal and cancel requests, responds, and calls user-defined functions for executing goals.
  * Handle result requests
  * Handle expired goals
  * Publish goal status array and feedback
  * Add `handle_accepted_callback` to ActionServer
  * Enable test using MultiThreadedExecutor (`#280 <https://github.com/ros2/rclpy/issues/280>`_)
  * Guard against failed take when taking action messages (`#281 <https://github.com/ros2/rclpy/issues/281>`_)
* Contributors: Jacob Perron

0.6.2 (2019-02-08)
------------------
* Added Waitable to callback group (`#265 <https://github.com/ros2/rclpy/issues/265>`_)
* Fixed flake8 error (`#263 <https://github.com/ros2/rclpy/issues/263>`_)
* Added HIDDEN_NODE_PREFIX definition to node.py (`#259 <https://github.com/ros2/rclpy/issues/259>`_)
* Added rclpy raw subscriptions (`#242 <https://github.com/ros2/rclpy/issues/242>`_)
* Added a test for invalid string checks on publishing (`#256 <https://github.com/ros2/rclpy/issues/256>`_)
* Contributors: AAlon, Jacob Perron, Joseph Duchesne, Michel Hidalgo, Shane Loretz

0.6.1 (2018-12-07)
------------------
* Added node graph functions (`#247 <https://github.com/ros2/rclpy/issues/247>`_)
* Filled ParameterEvent.msg with timestamp and node path name (`#252 <https://github.com/ros2/rclpy/issues/252>`_)
* Fixed spelling in documentation (`#251 <https://github.com/ros2/rclpy/issues/251>`_)
* Added Waitaible and wait set APIs (`#250 <https://github.com/ros2/rclpy/issues/250>`_)
* Updated rcl_wait_set_add\_* calls (`#248 <https://github.com/ros2/rclpy/issues/248>`_)
* Contributors: Brian, Dirk Thomas, Jacob Perron, Ross Desmond, Shane Loretz, Tully Foote, William Woodall

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
