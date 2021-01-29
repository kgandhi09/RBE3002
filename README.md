![Capture](https://user-images.githubusercontent.com/36654439/106231389-9ddbaf00-61bf-11eb-8465-a2e20f81080b.PNG)

# Unknown Environment Mapping: ROS
This project was part of undergraduate robotics RBE 3002 course. This goal of this project was to autonomously explore and navigate a turtlebot in an unkown environment(maze) and map that unknown area. The team used the Gmapping package, which provides laser based SLAM to localize the robot in the world, get the current explored map of the world in form of the occupancy grid and find an interesting frontiers to expand the map. Upon finding an interesting frontier, the team implemented and used A* algorithm to optimally drive to that frontier to furether explore the world. Finally, we identified when the map was complete and saved it. The team was not allowed to use the move_base package, thus we wrote our own move base packages.

## For more details, here's the link to the final report: https://drive.google.com/file/d/1R9blxKhp65IBZcF6x8dk4LUIcqoHrcWg/view?usp=sharing
