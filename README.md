# dqn_assets

```
LUA_PATH="/home/$USER/work/ml/dqn_assets/?.lua;/home/$USER/work/ml/dqn_kaixhin/?.lua;$LUA_PATH"
export LUA_PATH
source /home/$USER/work/ros-kinetic/kulbu_ws/devel/setup.bash
```

```
roslaunch kulbu_base sim.launch use_ekf:=false use_twowheels:=true robot:=kulbabu ns:=kulbabu0 world:=rat1

th main.lua -env rlenvs.Kulbabu -modelBody models.Kulbabu -height 1 -width 8 -duel true -doubleQ true -bootstraps 0 -memPriority rank -alpha 0.7 -betaZero 0.5 -PALpha 0 -gradClip 0 -hiddenSize 128 -valFreq 12000 -valSteps 3000

roslaunch kulbu_gazebo spawn_robot.launch use_ekf:=false use_twowheels:=true robot:=kulbabu ns:=kulbabu1 x:=0 y:=1
roslaunch kulbu_gazebo spawn_robot.launch use_ekf:=false use_twowheels:=true robot:=kulbabu ns:=kulbabu2 x:=0 y:=2
roslaunch kulbu_gazebo spawn_robot.launch use_ekf:=false use_twowheels:=true robot:=kulbabu ns:=kulbabu3 x:=1 y:=0
roslaunch kulbu_gazebo spawn_robot.launch use_ekf:=false use_twowheels:=true robot:=kulbabu ns:=kulbabu4 x:=2 y:=0

th main.lua -async A3C -env rlenvs.Kulbabu -modelBody models.Kulbabu -entropyBeta 0.001 -eta 0.0007 -momentum 0.99 -bootstraps 0 -batchSize 5 -doubleQ false -duel false -optimiser adam -steps 15000000 -tau 4 -memSize 20000 -epsilonSteps 10000 -bootstraps 0 -PALpha 0 -height 1 -width 8 -threads 4 -hiddenSize 128 -valFreq 12000 -valSteps 3000
```
