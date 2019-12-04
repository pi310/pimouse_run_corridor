<launch>
  <include file="$(find pimouse_ros)/launch/pimouse.launch" />
  <node pkg="pimouse_run_corridor" name="wall_around" type="wall_around.py" required="true" />
  <test test-name="test_wall_around" pkg="pimouse_run_corridor" type="travis_test_wall_around.py" />
</launch>
