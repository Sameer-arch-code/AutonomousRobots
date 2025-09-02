The bot moves between yellow and white lines. Yellow should be on left side. 

Steps:

1. Connect to your turtlebot and docker compose up.
2. Put your bot between lines to be followed.
3. Comment this line 'self.cv_timerYellow = self.create_timer(0.1, self.lineDetectionYellow, callback_group = cv_cb_group)'
4. Colcon build.
5. Run the node named - 'autorace_real' . Used the command : ros2 run autorace_real autorace_real
6. This will start printing - yellowLineFromCentreDistance: 'value'
7. You have copy 'value' and put it inside the variable - 'calib'
8. After this uncomment the 'self.cv_timerYellow = self.create_timer(0.1, self.lineDetectionYellow, callback_group = cv_cb_group)'
9. Colcon build.
10. Place an object right infront of bot (just 10 cm away)
11. Run the node.
12. Robot will wait for the object to be removed.
13. After removing object, just watch the beast move.




