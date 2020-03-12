## Implementing error checking for Panda with Frankx
**********
Operating Franka Emika Panda can lead to errors due to your code, motion planning or collision. Fortunately, there are libraries which can be installed to automatically recover from errors, avoiding damage to the robot and allowing the code to rerun following an error.
This tutorial uses ```Frankx```, which is a python wrapper for ```libfranka``` (which runs in C++). The library makes real-time trajectory generation easy, allowing the robot to respond to unforeseen events.

#### The following steps can be followed to implement error checking


1. Install the following:

    - [libfranka](https://github.com/frankaemika/libfranka)
    - [Reflexxes(trajectory-generator)](http://reflexxes.ws/)
    - [Eigen (transformation calculations)](http://eigen.tuxfamily.org/index.php?title=Main_Page)
    - [pybind11 (Python bindings)](https://github.com/pybind/pybind11)


&nbsp;
2.  Install ```frankx``` via:

```python
mkdir -p build
cd build
cmake -DReflexxes_ROOT_DIR=../RMLTypeII -DREFLEXXES_TYPE=ReflexxesTypeII -DBUILD_TYPE=Release ..
make -j4
make install
```
&nbsp;
3.  To recover from errors include the line:

```python
robot.recover_from_errors()
```

This allows the robot to automatically recover from errors by resetting the robot.
