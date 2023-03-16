^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix test_publisher linter for pydocstyle 6.2.2 (`#1063 <https://github.com/ros2/rclpy/issues/1063>`_) (`#1068 <https://github.com/ros2/rclpy/issues/1068>`_)
* Waitable should check callback_group if it can be executed. (`#1001 <https://github.com/ros2/rclpy/issues/1001>`_) (`#1015 <https://github.com/ros2/rclpy/issues/1015>`_)
* Contributors: mergify[bot], Cristóbal Arroyo, Tomoya Fujita

1.0.11 (2022-10-12)
-------------------
* Revert "Raise user handler exception in MultiThreadedExecutor. (`#984 <https://github.com/ros2/rclpy/issues/984>`_)" (`#1017 <https://github.com/ros2/rclpy/issues/1017>`_) (`#1021 <https://github.com/ros2/rclpy/issues/1021>`_)
* Contributors: Tomoya Fujita, mergify[bot]

1.0.10 (2022-09-12)
-------------------
* Raise user handler exception in MultiThreadedExecutor (`#984 <https://github.com/ros2/rclpy/issues/984>`_) (`#992 <https://github.com/ros2/rclpy/issues/992>`_)
* Contributors: Tomoya Fujita

1.0.9 (2022-09-01)
------------------
* Add __enter_\_ and __exit_\_ to Waitable (`#761 <https://github.com/ros2/rclpy/issues/761>`_) (`#998 <https://github.com/ros2/rclpy/issues/998>`_)
* Remove feedback callback when the goal has been completed (`#927 <https://github.com/ros2/rclpy/issues/927>`_) (`#933 <https://github.com/ros2/rclpy/issues/933>`_)
* Fix memory leak for get\_(publishers|subscriptions)_info_by_topic (`#955 <https://github.com/ros2/rclpy/issues/955>`_)
* Fix type annotation for get_parameters_by_prefix (`#964 <https://github.com/ros2/rclpy/issues/964>`_)
* Fix rclpy.duration.Duration.to_msg() losing precision (`#876 <https://github.com/ros2/rclpy/issues/876>`_) (`#917 <https://github.com/ros2/rclpy/issues/917>`_)
* Fix inverted error code for action client take (`#949 <https://github.com/ros2/rclpy/issues/949>`_)
* Fix memory leak for serialization (`#898 <https://github.com/ros2/rclpy/issues/898>`_)
* Fix crash on spinning raw subscription when publishes closes (`#827 <https://github.com/ros2/rclpy/issues/827>`_). (`#903 <https://github.com/ros2/rclpy/issues/903>`_)
* Contributors: Chen Lihui, Chris Lalancette, Erki Suurjaak, HyunSeok Kil, Jonathan Chapple, Shane Loretz, Tully Foote, Tomoya Fujita

1.0.8 (2022-01-31)
------------------
* Adding wake_executor to subscriber (`#855 <https://github.com/ros2/rclpy/issues/855>`_)
* Contributors: Leonardo Oliveira Wellausen

1.0.7 (2021-10-05)
------------------
* Only add one done callback to a future (`#816 <https://github.com/ros2/rclpy/issues/816>`_) (`#820 <https://github.com/ros2/rclpy/issues/820>`_)
* Contributors: Shane Loretz

1.0.6 (2021-08-31)
------------------
* Reject cancel request if failed to transit to CANCEL_GOAL state (`#791 <https://github.com/ros2/rclpy/issues/791>`_) (`#796 <https://github.com/ros2/rclpy/issues/796>`_)
* Break log function execution ASAP if configured severity is too high (`#776 <https://github.com/ros2/rclpy/issues/776>`_) (`#784 <https://github.com/ros2/rclpy/issues/784>`_)
* Contributors: Jacob Perron, Tomoya Fujita, ksuszka

1.0.5 (2021-01-05)
------------------
* [documentation] Use only True to avoid confusion in autodoc config
  `None` and `True` have the same meaning:
  https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html?highlight=autoclass#confval-autodoc_default_options
* [documentation] Document QoS profile constants
* Fix Enum not being comparable with ints in get_parameter_types service (`#644 <https://github.com/ros2/rclpy/issues/644>`_)
* Make sure to catch the ROSInterruptException when calling rate.sleep (`#650 <https://github.com/ros2/rclpy/issues/650>`_)
* Fix memory leak (`#643 <https://github.com/ros2/rclpy/issues/643>`_) (`#646 <https://github.com/ros2/rclpy/issues/646>`_)
* Destroy event handlers owned by publishers/subscriptions when calling publisher.destroy()/subscription.destroy() (`#603 <https://github.com/ros2/rclpy/issues/603>`_) (`#625 <https://github.com/ros2/rclpy/issues/625>`_)
* Use absolute parameter events topic name (`#612 <https://github.com/ros2/rclpy/issues/612>`_) (`#614 <https://github.com/ros2/rclpy/issues/614>`_)
* MultiThreadedExecutor spin_until_future complete should not continue waiting when the future is done (`#605 <https://github.com/ros2/rclpy/issues/605>`_) (`#626 <https://github.com/ros2/rclpy/issues/626>`_)
* Contributors: Chen Lihui Chris Lalancette, Gökçe Aydos, Ivan Santiago Paunovic, Jacob Perron, Tomoya Fujita

1.0.4 (2020-07-07)
------------------
* Wrap lines to shorten line length (`#586 <https://github.com/ros2/rclpy/issues/586>`_)
* Improve error message if rclpy C extensions are not found (`#580 <https://github.com/ros2/rclpy/issues/580>`_)
* Fix moved troubleshooting URL (`#579 <https://github.com/ros2/rclpy/issues/579>`_)
* Add resolved_name() method to publisher (`#568 <https://github.com/ros2/rclpy/issues/568>`_) (`#576 <https://github.com/ros2/rclpy/issues/576>`_)
* Contributors: Dirk Thomas, Shane Loretz

1.0.3 (2020-06-23)
------------------
* Fix executor wait_for_ready_callbacks returning None on shutdown (`#574 <https://github.com/ros2/rclpy/issues/574>`_) (`#583 <https://github.com/ros2/rclpy/issues/583>`_)
* Add ``topic_name`` property to Subscription (`#571 <https://github.com/ros2/rclpy/issues/571>`_) (`#582 <https://github.com/ros2/rclpy/issues/582>`_)
* Doc+fix ``rclpy_handle_get_pointer_from_capsule()`` (`#569 <https://github.com/ros2/rclpy/issues/569>`_) (`#570 <https://github.com/ros2/rclpy/issues/570>`_)
* Contributors: Audrow Nash, Shane Loretz

1.0.2 (2020-06-01)
------------------
* Protect access to global logging calls with a mutex (`#562 <https://github.com/ros2/rclpy/issues/562>`_)
* Ensure executors' spinning API handles shutdown properly (`#563 <https://github.com/ros2/rclpy/issues/563>`_)
* Contributors: Michel Hidalgo, William Woodall

1.0.1 (2020-05-18)
------------------
* Explicitly add DLL directories for Windows before importing (`#558 <https://github.com/ros2/rclpy/issues/558>`_)
* Contributors: Jacob Perron

1.0.0 (2020-05-12)
------------------
* Remove MANUAL_BY_NODE liveliness API (`#556 <https://github.com/ros2/rclpy/issues/556>`_)
* Fix bug that not to get expected data because use less timeout (`#548 <https://github.com/ros2/rclpy/issues/548>`_)
* Contributors: Barry Xu, Ivan Santiago Paunovic

0.9.1 (2020-05-08)
------------------
* Fix bad rmw_time_t to nanoseconds conversion. (`#555 <https://github.com/ros2/rclpy/issues/555>`_)
* Skip flaky timer test on windows (`#554 <https://github.com/ros2/rclpy/issues/554>`_)
* Cleanup rmw publisher/subscription on exception (`#553 <https://github.com/ros2/rclpy/issues/553>`_)
* Contributors: Ivan Santiago Paunovic, Miaofei Mei, Michel Hidalgo

0.9.0 (2020-04-29)
------------------
* Fix flaky test expecting wrong return type of rclpy_take (`#552 <https://github.com/ros2/rclpy/issues/552>`_)
* Fix warning about pytaken_msg maybe being uninitialized (`#551 <https://github.com/ros2/rclpy/issues/551>`_)
* Handle a failed rcl_take() call in rclpy_take() (`#550 <https://github.com/ros2/rclpy/issues/550>`_)
* Enforce a precedence for wildcard matching in parameter overrides (`#547 <https://github.com/ros2/rclpy/issues/547>`_)
* Feature/services timestamps (`#545 <https://github.com/ros2/rclpy/issues/545>`_)
* Add method to take with message_info (`#542 <https://github.com/ros2/rclpy/issues/542>`_)
* Ensure logging is initialized only once (`#518 <https://github.com/ros2/rclpy/issues/518>`_)
* Update includes to use non-entry point headers from detail subdir (`#541 <https://github.com/ros2/rclpy/issues/541>`_)
* Create a default warning for qos incompatibility (`#536 <https://github.com/ros2/rclpy/issues/536>`_)
* Add enclaves introspection method in ``Node`` (`#538 <https://github.com/ros2/rclpy/issues/538>`_)
* Rename rosidl_generator_c namespace to rosidl_runtime_c (`#540 <https://github.com/ros2/rclpy/issues/540>`_)
* Use f-string to fix flake8 warning (`#539 <https://github.com/ros2/rclpy/issues/539>`_)
* Don't persist node and context between tests (`#526 <https://github.com/ros2/rclpy/issues/526>`_)
* Avoid unsigned/signed comparison (`#535 <https://github.com/ros2/rclpy/issues/535>`_)
* Support for ON_REQUESTED_INCOMPATIBLE_QOS and ON_OFFERED_INCOMPATIBLE_QOS events (`#459 <https://github.com/ros2/rclpy/issues/459>`_)
* Switch to slightly more generic isinstance
* Add capability to publish serialized messages (`#509 <https://github.com/ros2/rclpy/issues/509>`_)
* Set context when creating Timer (`#525 <https://github.com/ros2/rclpy/issues/525>`_)
* Don't check lifespan on subscriber QoS (`#523 <https://github.com/ros2/rclpy/issues/523>`_)
* Deprecate set_parameters_callback API (`#504 <https://github.com/ros2/rclpy/issues/504>`_)
* Add env var to filter available RMW implementations (`#522 <https://github.com/ros2/rclpy/issues/522>`_)
* Fix object destruction order (`#497 <https://github.com/ros2/rclpy/issues/497>`_)
* Fixed flake8 rclpy test utilities (`#519 <https://github.com/ros2/rclpy/issues/519>`_)
* Fixes max_jitter calculation (`#512 <https://github.com/ros2/rclpy/issues/512>`_)
* Included get_available_rmw_implementations (`#517 <https://github.com/ros2/rclpy/issues/517>`_)
* Embolden warning about Client.call() potentially deadlocking (`#516 <https://github.com/ros2/rclpy/issues/516>`_)
* Enable test_get_publishers_subscriptions_info_by_topic() unit test for more rmw_implementations (`#511 <https://github.com/ros2/rclpy/issues/511>`_)
* Change sizes to Py_ssize_t (`#514 <https://github.com/ros2/rclpy/issues/514>`_)
* Rename rmw_topic_endpoint_info_array count to size (`#510 <https://github.com/ros2/rclpy/issues/510>`_)
* Implement functions to get publisher and subcription informations like QoS policies from topic name (`#454 <https://github.com/ros2/rclpy/issues/454>`_)
* Call init and shutdown thread safely (`#508 <https://github.com/ros2/rclpy/issues/508>`_)
* Support multiple "on parameter set" callbacks (`#457 <https://github.com/ros2/rclpy/issues/457>`_)
* Code style only: wrap after open parenthesis if not in one line (`#500 <https://github.com/ros2/rclpy/issues/500>`_)
* Add wrappers for RMW serialize and deserialize functions (`#495 <https://github.com/ros2/rclpy/issues/495>`_)
* Move logic for getting type support into a common function (`#492 <https://github.com/ros2/rclpy/issues/492>`_)
* Find test dependency rosidl_generator_py (`#493 <https://github.com/ros2/rclpy/issues/493>`_)
* Avoid reference cycle between Node and ParameterService (`#490 <https://github.com/ros2/rclpy/issues/490>`_)
* Avoid a reference cycle between Node and TimeSource (`#488 <https://github.com/ros2/rclpy/issues/488>`_)
* Fix typo (`#489 <https://github.com/ros2/rclpy/issues/489>`_)
* Handle unknown global ROS arguments (`#485 <https://github.com/ros2/rclpy/issues/485>`_)
* Fix the type annotation on get_parameters_by_prefix (`#482 <https://github.com/ros2/rclpy/issues/482>`_)
* Replace RuntimeError with new custom exception RCLError (`#478 <https://github.com/ros2/rclpy/issues/478>`_)
* Update constructor docstrings to use imperative mood (`#480 <https://github.com/ros2/rclpy/issues/480>`_)
* Use absolute topic name for rosout (`#479 <https://github.com/ros2/rclpy/issues/479>`_)
* Guard against unexpected action responses (`#474 <https://github.com/ros2/rclpy/issues/474>`_)
* Fix test_action_client.py failures (`#471 <https://github.com/ros2/rclpy/issues/471>`_)
* Enable/disable rosout logging in each node individually (`#469 <https://github.com/ros2/rclpy/issues/469>`_)
* Make use of rcutils log severity defined enum instead of duplicating code (`#468 <https://github.com/ros2/rclpy/issues/468>`_)
* Provide logging severity for string (`#458 <https://github.com/ros2/rclpy/issues/458>`_)
* Send feedback callbacks properly in send_goal() of action client (`#451 <https://github.com/ros2/rclpy/issues/451>`_)
* Contributors: Abhinav Singh, Alejandro Hernández Cordero, Barry Xu, Brian Marchi, Chris Lalancette, Dan Rose, Dirk Thomas, Donghee Ye, Emerson Knapp, Felix Divo, Ingo Lütkebohle, Ivan Santiago Paunovic, Jacob Perron, Jaison Titus, Miaofei Mei, Michel Hidalgo, Shane Loretz, Stephen Brawner, Steven! Ragnarök, Suyash Behera, Tully Foote, Werner Neubauer

0.8.3 (2019-11-18)
------------------
* Future invokes done callbacks when done (`#461 <https://github.com/ros2/rclpy/issues/461>`_)
* Make short key of a QoS policy accessible (`#463 <https://github.com/ros2/rclpy/issues/463>`_)
* Fix new linter warnings as of flake8-comprehensions 3.1.0 (`#462 <https://github.com/ros2/rclpy/issues/462>`_)
* Contributors: Dirk Thomas, Shane Loretz

0.8.2 (2019-11-13)
------------------
* Explicitly destroy a node's objects before the node. (`#456 <https://github.com/ros2/rclpy/issues/456>`_)
* Get proper parameters with prefixes without dot separator. (`#455 <https://github.com/ros2/rclpy/issues/455>`_)
* Fix import to use builtin_interfaces.msg (`#453 <https://github.com/ros2/rclpy/issues/453>`_)
* Add missing exec depend on rcl_interfaces (`#452 <https://github.com/ros2/rclpy/issues/452>`_)
* Contributors: Brian Marchi, Dirk Thomas, Steven! Ragnarök

0.8.1 (2019-10-23)
------------------
* Fix the unicode test string for opensplice rmw implementation (`#447 <https://github.com/ros2/rclpy/issues/447>`_)
* Expand test timeout to deflake rmw_connext (`#449 <https://github.com/ros2/rclpy/issues/449>`_)
* Support array parameter types (`#444 <https://github.com/ros2/rclpy/issues/444>`_)
* Make use of Clock class for throttling logs (`#441 <https://github.com/ros2/rclpy/issues/441>`_)
* Drop rclpy test_remove_ros_args_empty test case. (`#445 <https://github.com/ros2/rclpy/issues/445>`_)
* Add Rate (`#443 <https://github.com/ros2/rclpy/issues/443>`_)
* Action server: catch exception from user execute callback (`#437 <https://github.com/ros2/rclpy/issues/437>`_)
* Make cppcheck happy (`#438 <https://github.com/ros2/rclpy/issues/438>`_)
* Contributors: Brian Marchi, Jacob Perron, Michael Carroll, Michel Hidalgo, Shane Loretz

0.8.0 (2019-09-26)
------------------
* Take parameter overrides provided through the CLI. (`#434 <https://github.com/ros2/rclpy/issues/434>`_)
* Changelog version to master (`#410 <https://github.com/ros2/rclpy/issues/410>`_)
* Remove deprecated QoS functionality (`#431 <https://github.com/ros2/rclpy/issues/431>`_)
* Remove comment (`#432 <https://github.com/ros2/rclpy/issues/432>`_)
* Provide subscription count from Publisher `#418 <https://github.com/ros2/rclpy/issues/418>`_ (`#429 <https://github.com/ros2/rclpy/issues/429>`_)
* Raise custom error when node name is not found (`#413 <https://github.com/ros2/rclpy/issues/413>`_)
* Timer uses ROS time by default (`#419 <https://github.com/ros2/rclpy/issues/419>`_)
* Fix _rclpy.c formatting. (`#421 <https://github.com/ros2/rclpy/issues/421>`_)
* Fail on invalid and unknown ROS specific arguments (`#415 <https://github.com/ros2/rclpy/issues/415>`_)
* Force explicit --ros-args in cli args. (`#416 <https://github.com/ros2/rclpy/issues/416>`_)
* Make Future result() and __await_\_ raise exceptions (`#412 <https://github.com/ros2/rclpy/issues/412>`_)
* Use of -r/--remap flags where appropriate. (`#411 <https://github.com/ros2/rclpy/issues/411>`_)
* Awake waitables on shutdown, check if context is valid (`#403 <https://github.com/ros2/rclpy/issues/403>`_)
* Accept tuples as parameter arrays (`#389 <https://github.com/ros2/rclpy/issues/389>`_)
* Adapt to '--ros-args ... [--]'-based ROS args extraction (`#405 <https://github.com/ros2/rclpy/issues/405>`_)
* Replace 'NULL == ' with ! (`#404 <https://github.com/ros2/rclpy/issues/404>`_)
* Declaring 'use_sim_time' when attaching node to time source. (`#396 <https://github.com/ros2/rclpy/issues/396>`_)
* Adding ignore_override parameter to declare_parameter(s). (`#392 <https://github.com/ros2/rclpy/issues/392>`_)
* fix missing 'raise'
* Adding get_parameters_by_prefix method to Node. (`#386 <https://github.com/ros2/rclpy/issues/386>`_)
* remove whitespace (`#385 <https://github.com/ros2/rclpy/issues/385>`_)
* Added clients by node implementation from rcl (`#383 <https://github.com/ros2/rclpy/issues/383>`_)
* Fix time conversion for a big nanoseconds value (`#384 <https://github.com/ros2/rclpy/issues/384>`_)
* Allowing parameter declaration without a given value. (`#382 <https://github.com/ros2/rclpy/issues/382>`_)
* Make flake8 happy on windows (`#381 <https://github.com/ros2/rclpy/issues/381>`_)
* Rename QoS*Policy enum's to ``*Policy`` (`#379 <https://github.com/ros2/rclpy/issues/379>`_)
* Fixing namespace expansion for declare_parameters. (`#377 <https://github.com/ros2/rclpy/issues/377>`_)
* Use params from node ``'/**'`` from parameter YAML file (`#370 <https://github.com/ros2/rclpy/issues/370>`_)
* [executors] don't convert a timeout_sec to nsecs (`#372 <https://github.com/ros2/rclpy/issues/372>`_)
* Fix API documentation related to ROS graph methods (`#366 <https://github.com/ros2/rclpy/issues/366>`_)
* Treat warnings as test failures and fix warnings (`#365 <https://github.com/ros2/rclpy/issues/365>`_)
* Refactored _rclpy.rclpy_get_rmw_qos_profile to return dictionary instead of QoSProfile (`#364 <https://github.com/ros2/rclpy/issues/364>`_)
* Contributors: Brian Marchi, Christian Rauch, Daniel Stonier, Daniel Wang, Geno117, Jacob Perron, Juan Ignacio Ubeira, Michel Hidalgo, Scott K Logan, Shane Loretz, Siddharth Kucheria, Vinnam Kim, William Woodall, ivanpauno, suab321321

0.7.6 (2019-08-28)
------------------
* Fix missing raise (`#390 <https://github.com/ros2/rclpy/pull/390>`_)
* Fix time conversion for big nanoseconds value (`#384 <https://github.com/ros2/rclpy/pull/384>`_)
* Contributors: Daniel Wang, Vinnam Kim

0.7.5 (2019-08-01)
------------------
* Updated to use params from node ``'/**'`` from parameter YAML file. (`#399 <https://github.com/ros2/rclpy/issues/399>`_)
* Updated to declare 'use_sim_time' when attaching node to time source. (`#401 <https://github.com/ros2/rclpy/issues/401>`_)
* Fixed an errant conversion to nsecs in executors timeout.` (`#397 <https://github.com/ros2/rclpy/issues/397>`_)
* Fixed parameter handling issues. (`#394 <https://github.com/ros2/rclpy/issues/394>`_)
  * Fixing namespace expansion for declare_parameters. (`#377 <https://github.com/ros2/rclpy/issues/377>`_)
  * Allowing parameter declaration without a given value. (`#382 <https://github.com/ros2/rclpy/issues/382>`_)
* Contributors: Juan Ignacio Ubeira, Scott K Logan

0.7.4 (2019-06-12)
------------------
* Fix API documentation related to ROS graph methods (`#366 <https://github.com/ros2/rclpy/issues/366>`_)
* Contributors: Jacob Perron

0.7.3 (2019-05-29)
------------------
* Rename parameter options (`#363 <https://github.com/ros2/rclpy/issues/363>`_)
  * rename the initial_parameters option to parameter_overrides
  * rename automatically_declare_initial_parameters to automatically_declare_parameters_from_overrides
  * update allow_undeclared_parameters docs
* Consolidate create_publisher arguments (`#362 <https://github.com/ros2/rclpy/issues/362>`_)
* Enforcing parameter ranges. (`#357 <https://github.com/ros2/rclpy/issues/357>`_)
* Initialize QoSProfile with values from rmw_qos_profile_default (`#356 <https://github.com/ros2/rclpy/issues/356>`_)
* Contributors: Dirk Thomas, Emerson Knapp, Juan Ignacio Ubeira, William Woodall

0.7.2 (2019-05-20)
------------------
* Add convenience name translations for use by commandline utilities etc. (`#352 <https://github.com/ros2/rclpy/issues/352>`_)
* Wait for nodes to discover each other in test_action_graph.py (`#354 <https://github.com/ros2/rclpy/issues/354>`_)
* Destroy publishers after test is done (`#355 <https://github.com/ros2/rclpy/issues/355>`_)
* Create RLock() early to avoid exception at shutdown (`#351 <https://github.com/ros2/rclpy/issues/351>`_)
* Fix qos event argument being wrapped in list. It shouldn't have been (`#349 <https://github.com/ros2/rclpy/issues/349>`_)
* Parameter flexibility enhancements (`#347 <https://github.com/ros2/rclpy/issues/347>`_)
* Update troubleshooting reference to index.ros.org (`#348 <https://github.com/ros2/rclpy/issues/348>`_)
* Update test since unicode characters are allowed now (`#346 <https://github.com/ros2/rclpy/issues/346>`_)
* Parameter handling improvements. (`#345 <https://github.com/ros2/rclpy/issues/345>`_)
* Encourage users to always provide a QoS history depth (`#344 <https://github.com/ros2/rclpy/issues/344>`_)
* QoS - API and implementation for Liveliness and Deadline event callbacks (`#316 <https://github.com/ros2/rclpy/issues/316>`_)
* Ignore flake8 error 'imported but unused' (`#343 <https://github.com/ros2/rclpy/issues/343>`_)
* Contributors: Dirk Thomas, Emerson Knapp, Jacob Perron, Juan Ignacio Ubeira, Michael Carroll, Michel Hidalgo, Shane Loretz

0.7.1 (2019-05-08)
------------------
* Update tests to include namespace in ROS types (`#294 <https://github.com/ros2/rclpy/issues/294>`_)
* Capsule available at self.handle (`#340 <https://github.com/ros2/rclpy/issues/340>`_)
* Wake executor when entities created or destroyed (`#336 <https://github.com/ros2/rclpy/issues/336>`_)
* Setting automatic declaration for initial parameters to False. (`#339 <https://github.com/ros2/rclpy/issues/339>`_)
* Improve signal handling (`#338 <https://github.com/ros2/rclpy/issues/338>`_)
* Parameter API enhancements (`#325 <https://github.com/ros2/rclpy/issues/325>`_)
* QoS - Expose the assert_liveliness API for Publishers and Nodes (`#313 <https://github.com/ros2/rclpy/issues/313>`_)
* Minimal change to build against new rcl API (`#305 <https://github.com/ros2/rclpy/issues/305>`_)
* Remove extra references to node handle (`#335 <https://github.com/ros2/rclpy/issues/335>`_)
* API updates for RMW preallocation work. (`#337 <https://github.com/ros2/rclpy/issues/337>`_)
* Make pub/sub/cli/srv/etc lists use @property on node (`#333 <https://github.com/ros2/rclpy/issues/333>`_)
* Ignore ValueError in SignalHandlerGuardCondition.__del_\_ (`#334 <https://github.com/ros2/rclpy/issues/334>`_)
* Use new test interface definitions (`#332 <https://github.com/ros2/rclpy/issues/332>`_)
* Thread safe node.destroy\_* (`#319 <https://github.com/ros2/rclpy/issues/319>`_)
* Make ``destroy_node`` thread safe (`#330 <https://github.com/ros2/rclpy/issues/330>`_)
* Remove most of the timing checks in test_executor (`#329 <https://github.com/ros2/rclpy/issues/329>`_)
* Prevent rcutils_log from accessing invalid memory (`#326 <https://github.com/ros2/rclpy/issues/326>`_)
* Wait set uses pointers to rcl types not rcl->impl types (`#324 <https://github.com/ros2/rclpy/issues/324>`_)
* QoS - Expose Lifespan, Deadline, and Liveliness policy settings (`#312 <https://github.com/ros2/rclpy/issues/312>`_)
* Remove __eq_\_ and __hash_\_ from Subscription (`#323 <https://github.com/ros2/rclpy/issues/323>`_)
* Fix subscription pycapsule not being destroyed (`#320 <https://github.com/ros2/rclpy/issues/320>`_)
* Make destroy_subscription thread safe (`#318 <https://github.com/ros2/rclpy/issues/318>`_)
* enforce correct message type is passed to various API (`#317 <https://github.com/ros2/rclpy/issues/317>`_)
* Every executor gets its own SIGINT guard condition (`#308 <https://github.com/ros2/rclpy/issues/308>`_)
* add missing error handling and cleanup (`#315 <https://github.com/ros2/rclpy/issues/315>`_)
* Rename action state transitions (`#300 <https://github.com/ros2/rclpy/issues/300>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Emerson Knapp, Jacob Perron, Juan Ignacio Ubeira, Michael Carroll, Michel Hidalgo, Shane Loretz, Thomas Moulard

0.7.0 (2019-04-14)
------------------
* Added action graph API. (`#306 <https://github.com/ros2/rclpy/issues/306>`_)
* Added timeout to executor_spin_until_future_complete. (`#301 <https://github.com/ros2/rclpy/issues/301>`_)
* Refactored QoS Python-C conversion into less error-prone pattern (pre-QoS, standalone). (`#307 <https://github.com/ros2/rclpy/issues/307>`_)
* Set QoS profile to default values to future-proof against uninitialized data if new fields are added
* Fixed executor bug by refreshing nodes when executor is woken. (`#310 <https://github.com/ros2/rclpy/issues/310>`_)
* Updated so executor exits immediately when shut down. (`#309 <https://github.com/ros2/rclpy/issues/309>`_)
* Updated to use rosgraph_msgs.msg.Clock for TimeSource. (`#304 <https://github.com/ros2/rclpy/issues/304>`_)
* Added param callback to time_source. (`#297 <https://github.com/ros2/rclpy/issues/297>`_)
* Updated tests to pass with numpy arrays. (`#292 <https://github.com/ros2/rclpy/issues/292>`_)
* Improved error handling to avoid memory leaks in C extension. (`#278 <https://github.com/ros2/rclpy/issues/278>`_)
* Fixed sigint guard condition's lifecycle bug. (`#288 <https://github.com/ros2/rclpy/issues/288>`_)
  Updated to use ament_target_dependencies where possible. (`#286 <https://github.com/ros2/rclpy/issues/286>`_)
* Improved documentation. (`#277 <https://github.com/ros2/rclpy/issues/277>`_)
  * Document node.py.
  * Fix C extension documentation.
  * Document init, shutdown, and spinning.
  * Document Publisher and Subscription.
  * Document Client and Service.
  * Add warnings to constructors of client and service.
  * Document executors and callback groups.
  * Use typing,TYPE_CHECKING variable for condition imports used by annotations.
  * Add instructions for building docs to README.
  * Clarify doc briefs for graph discovery functions.
* Added RcutilsLogger.warning. (`#284 <https://github.com/ros2/rclpy/issues/284>`_)
* Changed logger.warn (deprecated) to logger.warning. (`#283 <https://github.com/ros2/rclpy/issues/283>`_)
* Updated to use separated action types. (`#274 <https://github.com/ros2/rclpy/issues/274>`_)
* Updated to guard against failed take when taking action messages. (`#281 <https://github.com/ros2/rclpy/issues/281>`_)
* Enabled test using MultiThreadedExecutor. (`#280 <https://github.com/ros2/rclpy/issues/280>`_)
* Added ActionServer. (`#270 <https://github.com/ros2/rclpy/issues/270>`_)
* Changed error raised by executor dict interface to KeyError. (`#276 <https://github.com/ros2/rclpy/issues/276>`_)
* Abstracted type conversions into functions (`#269 <https://github.com/ros2/rclpy/issues/269>`_)
* Fixed Node's reference to executor. (`#275 <https://github.com/ros2/rclpy/issues/275>`_)
* Updated to enforce UTF8 argv on rclpy.init(). (`#273 <https://github.com/ros2/rclpy/issues/273>`_)
* Fixed Executor not executing tasks if there are no ready entities in the wait set. (`#272 <https://github.com/ros2/rclpy/issues/272>`_)
* Replaced PyUnicode_1BYTE_DATA() with PyUnicode_AsUTF8(). (`#271 <https://github.com/ros2/rclpy/issues/271>`_)
* Added Action Client. (`#262 <https://github.com/ros2/rclpy/issues/262>`_)
* Updated to pass context to wait set. (`#258 <https://github.com/ros2/rclpy/issues/258>`_)
* Added Waitable to callback group. (`#265 <https://github.com/ros2/rclpy/issues/265>`_)
* Fixed flake8 error. (`#263 <https://github.com/ros2/rclpy/issues/263>`_)
* Added HIDDEN_NODE_PREFIX definition to node.py. (`#259 <https://github.com/ros2/rclpy/issues/259>`_)
* Added rclpy raw subscriptions. (`#242 <https://github.com/ros2/rclpy/issues/242>`_)
* Added a test for invalid string checks on publishing. (`#256 <https://github.com/ros2/rclpy/issues/256>`_)
* Contributors: AAlon, Dirk Thomas, Emerson Knapp, Jacob Perron, Joseph Duchesne, Michel Hidalgo, Shane Loretz, Vinnam Kim, Wei Liu, William Woodall, ivanpauno

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
