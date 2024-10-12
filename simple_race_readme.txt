1. git clone https://github.com/amazon-archives/aws-robomaker-sample-application-deepracer.git

2. git checkout -b ros2 origin/ros2

3. 修改simulation_ws/src/deepracer_simulation/launch/racetrack_with_racecar.launch.py文件如下
diff --git a/simulation_ws/src/deepracer_simulation/launch/racetrack_with_racecar.launch.py b/simulation_ws/src/deepracer_simulation/launch/racetrack_with_racecar.launch.py
index 945372c..35dc24c 100755
--- a/simulation_ws/src/deepracer_simulation/launch/racetrack_with_racecar.launch.py
+++ b/simulation_ws/src/deepracer_simulation/launch/racetrack_with_racecar.launch.py
@@ -14,20 +14,22 @@
 # SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 
 import os
+from ament_index_python.packages import get_package_share_directory
 import sys
 sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
 sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa
 
 import launch
-from launch_ros import get_default_launch_description
+# from launch_ros import get_default_launch_description
 import launch_ros.actions
-from ament_index_python.packages import get_package_share_directory
+# from ament_index_python.packages import get_package_share_directory
+# from ament_index_python.packages import get_package_share_directory
 
 def generate_launch_description():
 
     gui = launch.actions.DeclareLaunchArgument(
              'gui',
-             default_value='false',
+             default_value='true',
              description='Argument for GUI Display',
         )

4.  touch simulation_ws/src/deepracer_interpreter/COLCON_IGNORE
    touch simulation_ws/src/deepracer_msgs/COLCON_IGNORE
    touch simulation_ws/src/sagemaker_rl_agent/COLCON_IGNORE

5. source /opt/ros/humble/setup.sh
   source /usr/share/gazebo/setup.sh

6. colcon build

7. source install/setup.sh

8. ros2 launch deepracer_simulation racetrack_with_racecar.launch.py

9. ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=racer_car/cmd_vel