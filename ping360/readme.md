## Build The package in teh workspace directory

```bash
colcon build
```
## Source the package

```bash
source install/setup.bash
```
## Run

```bash
ros2 run ping360_sonar ping360.py
```
## If you encounter this error:
```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```
This means You're trying to run ROS 2 Jazzy (which uses Python 3.12), but your environment is trying to load modules using Python 3.13! it cxan happen if you are inside a virtual environment so deactivate the virtual environment:

```bash
conda deactivate
```
Then source your ROS 2 environment again:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash  # If your workspace is built
```
Than run
```bash
ros2 run ping360_sonar ping360.py
```
### !!! Now you are facing this error?

```
ModuleNotFoundError: No module named 'brping.definitions'
```
to solve this yiou just need to run the following

```bash
pip3 install --user bluerobotics-ping --upgrade --break-system-packages
```
Also run
```bash
git clone --single-branch --branch deployment https://github.com/bluerobotics/ping-python.git
cd ping-python
python setup.py install --user
```
## warning you may need to use --break-system-packages option otherwise it wont install as your environment maybe procted and externally managed.
### !!! For related discussion and potential fixes, see this Blue Robotics forum thread:
👉 [Module error in ping-python – Blue Robotics Forum](https://discuss.bluerobotics.com/t/module-error-in-ping-python/8128)


## Now run
```bash
ros2 run ping360_sonar ping360.py
```

### It should work fine if you encounter the following error:
```
ImportError: libpython3.13.so.1.0: cannot open shared object file: No such file or directory
```
it means your ROS 2 (Jazzy or humble or other version) uses Python 3.12 (/opt/ros/jazzy/...)

Your custom message ping360_sonar_msgs was previously built with Python 3.13 so you need to do the following
Just run: 
```bash
source /opt/ros/jazzy/setup.bash #your ros2 version should be replaced by jazzy, if you are using humble use source /opt/ros/humble/setup.bash
```
```bash
cd ~/Downloads/bluerov_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
ros2 run ping360_sonar ping360.py
```

This should work fine and you should see the soner data. The data field from the Ping360 sonar, a byte array of intensity values (0–255), represents the strength of the echo received from each direction and distance (range bin). With this, you can do a lot of useful things in underwater robotics and perception.
