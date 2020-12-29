# Reward-Criteria-Impact-on-RL

In this article, we study the reward criteria impact on the performance of reinforcement learning agent for autonomous navigation. The reward criteria chosen is based on the percentage of positive and negative rewards, which can be received by an agent. Based on this reward criteria three classes are formed, 'Balanced Class', 'Skewed Positive Class' and 'Skewed Negative Class'. 

# Task and Environment
Point goal navigation Task in gazebo simulation.
![alt text](https://github.com/aveen-d/Reward-Criteria-Impact-on-RL/blob/main/env4.png)

# Steps to run before performing experiment
1. Setup the turtlebot3, ROS 2 (Dashing) according to the manual. https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup .
2. Setup the turtlebot3 Simulation and Machine Learning Packages. ( Step 6 and Step 9 in the above manual)
3. Replace the dqn_agent.py and dqn_environment.py file in the machine learning package of turtlebot3 with the files mentioned in this repository.

# Experiment
After replacing the files, follow steps mentioned in  section 9.3.2  in the manual, https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#machine-learning .

# Results
Based on the experiments, the skewed negative class and balanced class are able to learn 74.6\% and 72.6\% respectively and a steady increase in average cumulative rewards has been observed. On the other hand, the skewed positive class did not show any steady improvement in the average cumulative rewards earned even after training for over a large number of episodes.

