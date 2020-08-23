# patch usage

### 20200822_change_namespace_to_enemy_bot.patch

this patche is to work this repository as enemy_bot, on blue side in simulation environment.
- 動作検証: ROS kinetic docker環境で実施 https://github.com/seigot/burger_war_docker_trial

```
cd ~/catkin_ws/src
sudo rm -r burger_war* obstacle_detector                              # delete old directory
git clone https://github.com/seigot/burger_war burger_war        # clone your repository
git clone https://github.com/seigot/burger_war burger_war_seigot # clone this repository for enemy_bot
git clone https://github.com/tysik/obstacle_detector.git         # if necessary
cd burger_war_seigot
git checkout ecc49420838007be362b0df368fdbeb33dbe99c4            # if necessary
patch -p1 < autotest/patch/20200822_change_namespace_to_enemy_bot.patch  # apply patch
cd ~/catkin_ws
catkin clean --yes   # if necessary, delete old devel/build/log directory
catkin build
bash devel/setup.sh
```

after build success. then...

```
cd ~/catkin_ws/src/burger_war
bash scripts/sim_with_judge.sh
bash scripts/start.sh -l 4           # your robot
cd ~/catkin_ws/src/burger_war_seigot
bash scripts/start.sh -l 4           # enemy robot
```
