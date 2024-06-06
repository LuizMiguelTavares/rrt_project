requirements

gmapping
amcl

gprof graph example- gprof ./RRT_simple gmon.out | gprof2dot -s | dot -Gdpi=200 -Tpng -o output.png

Command to syncronize the date with the master frome the pioneer

sudo ntpdate 192.168.0.100


Video order:
- RRT_simple with obstacle
- RRT_star
- RRT_simple
- RRT_star with obstacle local minimum
- RRT_star_with_obstacle_50_50
- RRT_star_with_obstacle_big_2
