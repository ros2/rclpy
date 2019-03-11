# rclpy
ROS Client Library for the Python language.

## Building documentation

Documentation can be built for `rclpy` using [Sphinx](http://www.sphinx-doc.org/en/master/).

#### Install dependencies

    sudo apt install python3-sphinx python3-pip
    sudo -H pip3 install sphinx_autodoc_typehints

#### Build

    cd rclpy/docs
    make html
