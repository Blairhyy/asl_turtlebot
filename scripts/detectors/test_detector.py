from numpy.linalg.linalg import det
import tensorflow.compat.v1 as tf

"""

    <arg name="detector" default="detector_mobilenet.py"/>

<node pkg="asl_turtlebot" type="supervisor.py" name="turtlebot_supervisor" output="screen" />
"""
tf.disable_v2_behavior()

conf = tf.ConfigProto()
conf.gpu_options.allow_growth = True
detection_graph = tf.Graph()
with detection_graph.as_default():
    with tf.io.gfile.GFile('../../tfmodels/stop_signs_gazebo.pb', 'rb') as f:
        od_graph_def = tf.GraphDef()
        od_graph_def.ParseFromString(f.read())
        tf.import_graph_def(od_graph_def,name='')
        print("loaded")
tf.print(tf.get_default_graph())
"""
        <launch> 
  <arg name="sim" default="true" /> 
  <arg name="gui" default="true" /> 
  <arg name="gmapping" default="true" /> 
  <arg name="rviz" default="" /> 
  <arg name="world" default="" /> 
  <arg name="detector" default="" /> 
  <arg name="navigator" default="" /> 
  <arg name="publish_state" default="true" /> 
  <arg name="x_pos" default="0.0" />
  <arg name="y_pos" default="0.0" />
  <arg name="z_pos" default="0.0" />

  <rosparam param="sim"   subst_value="true">$(arg sim)</rosparam>
  <rosparam param="map"   >true</rosparam>
  <rosparam param="rviz" subst_value="true">$(arg rviz)</rosparam>

  <!-- ###### launch the robot and possible state publisher ############### -->
  <arg name="model" default="asl_turtlebot" doc="model type [burger, waffle]"/>
  <param name="robot_description" command="$(find xacro)/xacro $(find asl_turtlebot)/urdf/$(arg model).urdf.xacro" if="$(eval '/' not in model)" />
  <param name="robot_description" command="$(find xacro)/xacro $(arg model)" unless="$(eval '/' not in model)" />

  <group if="$(arg publish_state)" >
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
      <param name="publish_frequency" type="double" value="30.0" />
    </node>
  </group>

  <!-- ###################### launch simulation if specified ############## -->
  <group if="$(arg sim)">
    <!-- ######### select the world by short name or by path ############ -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
      <arg name="use_sim_time" value="true"/>
      <arg name="world_name" value="$(find asl_turtlebot)/world/$(arg world).world" if="$(eval '/' not in world)" />
      <arg name="world_name" value="$(arg world)" unless="$(eval '/' not in world)"/>
      <arg name="paused" value="false"/>
      <arg name="debug" value="false"/>
      <arg name="gui" value="$(arg gui)"/>
    </include>

    <!-- ########### spawn the robot in simulation ################# -->
    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model turtlebot3_burger -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
  </group>

  <!-- ###################### launch the configured gmapping ############## -->
  <group if="$(arg gmapping)">
    <include file="$(find asl_turtlebot)/launch/config/gmapping_config.launch" />
  </group>

  <!-- ################ unconditonally useful scripts ##################### -->
  <node pkg="asl_turtlebot" type="goal_commander.py" name="goal_commander" />

  <!-- ################ conditionally launched scripts #################### -->
  <group if="$(eval len(detector) > 0)">
    <node pkg="asl_turtlebot" type="$(arg detector)" 
      name="turtlebot_detector" />
  </group>
  <group if="$(eval len(navigator) > 0)">
    <node pkg="asl_turtlebot" type="$(arg navigator)" name="turtlebot_navigator" output="screen" />
  </group>

  <!-- ###################### conditionally launch rviz ################### -->
  <group if="$(eval len(rviz) > 0)">
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find asl_turtlebot)/rviz/$(arg rviz).rviz"/>
  </group>

</launch>

        """