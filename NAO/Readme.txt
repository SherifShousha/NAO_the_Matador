To run the nao_matador do the following:

1. Turn on the robot and wait for it to initialize
2. Start terminals and in each run one of the following programs
3. Run the NAO controllers
    $ roslaunch nao_bringup nao_full_py.launch
    $ roslaunch nao_apps leds.launch
    $ roslaunch nao_apps tactile.launch
    $ roslaunch nao_apps speech.launch
4. Run the nao_matador servers
    $ rosrun nao_matador audio_player_server.py $NAO_IP 9559
    $ rosrun nao_matador motion_server.py $NAO_IP 9559
5. Run the nao_matador modules
    $ rosrun nao_matador module_vision
    $ rosrun nao_matador module_leds
    $ rosrun nao_matador module_bumpers
    $ rosrun nao_matador module_speech
    $ rosrun nao_matador module_walking
6. Run the state machine (It will only work if all of the modules are running)
    $ rosrun nao_matador corazon.py