# rclpy
ROS Client Library for the Python language.


## Documentation

Examples and tutorials for using RCLPy can be found at [docs.ros.org](https://docs.ros.org/). 
The latest function-level API documentation for RCLPy can be found in our [API documentation](https://docs.ros.org/en/rolling/p/rclpy/) or via searching for RCLPy on [ROS Index](https://index.ros.org/p/rclpy/). 

## Building documentation

Documentation can be built for `rclpy` using [Sphinx](http://www.sphinx-doc.org/en/master/), or accessed [online](https://docs.ros.org/en/rolling/p/rclpy/).

For building documentation, you need an installation of ROS 2.

#### Install dependencies

    sudo apt install \
      python3-sphinx \
      python3-sphinx-autodoc-typehints \
      python3-sphinx-rtd-theme

#### Build

Source your ROS 2 installation, for example:

    . /opt/ros/rolling/setup.bash

Build code:

    mkdir -p rclpy_ws/src
    cd rclpy_ws/src
    git clone https://github.com/ros2/rclpy.git
    cd ..
    colcon build --symlink-install

Source workspace and build docs:

    source install/setup.bash
    cd src/rclpy/rclpy/docs
    make html

# Contributions

Please read the ROS 2 Contribution Guide available on [docs.ros.org](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing.html#) before contributing.
If you are looking for a place to start consider checking out our issues tagged [`help wanted`](https://github.com/ros2/rclpy/issues?q=is%3Aissue%20state%3Aopen%20label%3A%22help%20wanted%22) and [`good first issue`](https://github.com/ros2/rclpy/issues?q=is%3Aissue%20state%3Aopen%20label%3A%22good%20first%20issue%22)
Pull request reviews from new community members are welcomed and encouraged!
Please check out the pull request, build it, and run the tests, before reviewing the code.
For new feature contributions, please consider filing an issue with the `enhancement` tag first, so we can discuss it ahead of time.
Additional support and contribution resources are listed on our [documentation landing page.](https://docs.ros.org/)
