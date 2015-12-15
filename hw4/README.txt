COMS W4733 Computational Aspects of Robotics 2015 hw4 part1
a) Group members:

   Team number: 24
   Team leader: Chia-Jung Lin (cl3295)
   Team members: Cheng Zhang (cz2398), Ming-Ching Chu (mc4107)


b) Files:

    /--+--README
       +—-hw4_part1.m
       +—-dijkstra.m
       +—-isIntersect.m
       +—-convex_hull.m
       +—-is_turn_left.m
       +—-example.png



c) How to run:
    To run our program, simply run hw4_part1().

d) Details:

1. We need hw4_world_and_obstacles_convex.txt and hw4_start_goal.txt as specified in the homework description.

2. we set our robot model to be a square of 0.35 meter.

3. reflection algorithm: Please refer to the ‘Grow Obstacles’ section in hw4_part1.m, especially Obj_grow() function in hw4_part1.m.

4. convex hull algorithm: We implemented the Graham’s algorithm. Please refer to convex_hull.m and is_turn_left.m.

5. Dijkstra algorithm: Please refer to dijkstra.m

6. graph annotation:
   We put text labels at start and goal.
   Safe path is marked in red.
   Original obstacles are marked in cyan.
   Grown obstacles are marked in black.
   Visibility graph is marked in blue. (We also remove edges that are too close to wall from visibility graph.)

7. It takes about 5 seconds to run our program :( Please be patient. Thank you!
