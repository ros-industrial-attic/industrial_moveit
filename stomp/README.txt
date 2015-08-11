To try out the 2D simulations:

1) Compile:

rosmake stomp

2) Run stomp on the problem defined in stomp/test/stomp_2d_test.yaml:

roslaunch stomp stomp_2d_test.launch

3) Visualize the results:

cd ~/.ros/stomp_test
rosrun stomp stomp_2d_test_plot.py

The parameters for the 2D cost function and STOMP are in the stomp_2d_test.yaml
file, feel free to experiment with those.
