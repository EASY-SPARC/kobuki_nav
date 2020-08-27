# kobuki_nav

Downloaded kobuki_desktop melodic branch from: https://github.com/yujinrobot/kobuki_desktop  
Downloaded kobuki melodic branch from: https://github.com/yujinrobot/kobuki.git  
Installed packages with:  
`sudo apt install ros-melodic-kobuki-*`  
`sudo apt install ros-melodic-yocs-*`  
`sudo apt install ros-melodic-ecl-streams`  

Closer to goal, kobuki has unexpected behaviour. The problem could be caused by global planner that wont predict the frame displacement, generating new trajectories with points under the kobuki, while expecting the base frame to reach goal.
