# patch usage

### 20200822_change_namespace_to_enemy_bot.patch

```
cd ~/catkin_ws/src
rm -r burger_war*                                                # delete burger_war directory
git clone https://github.com/seigot/burger_war burger_war        # clone your repository
git clone https://github.com/seigot/burger_war burger_war_seigot # clone this repository for enemy_bot
git clone https://github.com/tysik/obstacle_detector.git         # if necessary
cd burger_war_seigot
patch -p1 < autotest/patch/20200822_change_namespace_to_enemy_bot.patch  # apply patch
cd ~/catkin_ws
catkin build              # if necessary, delete old devel/build/log directory
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