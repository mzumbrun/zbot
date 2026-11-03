misc commands

to print a file
cd ~/zbot_ws/src/zbot/docs/runbooks
pandoc so101_arm.md -s -o ~/so101_arm.html --metadata title="SO-101 arms"
chromium ~/so101_arm.html        # or: firefox ~/so101_arm.html

to set default printer
lpoptions -d HP_Color_LaserJet_MFP_M283cdw_0611CD

now can print by
lp <file>
or with options set first
lpoptions -p HP_Color_LaserJet_MFP_M283cdw_0611CD -o sides=two-sided-long-edge


=====================

Terminal 1: leader node

bash
ros && source ~/zbot_ws/install/setup.bash
ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_leader_1

Terminal 2: follower bridge (leave the torque off and don’t enable it)

bash
ros && source ~/zbot_ws/install/setup.bash
ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_follower_1

Terminal 3: offsets tool

bash
ros && source ~/zbot_ws/install/setup.bash
cd ~/zbot_ws/src/zbot
python3 arm_lab/leader_offsets.py --write
Capture 1: pose both arms the same by eye, in a mid pose with nothing on a stop. Press Enter.
Capture 2: pose both arms the same again, but differently from capture 1, for example wrist up instead of down or the elbow more bent. Press Enter.
It prints the result table and writes zbot_leader_1.yaml.

==========================
No need to move files one by one. tar extracts the whole thing in one go, putting each file in its right place, because the folders are stored inside the tarball.

bash
cd ~/zbot_ws/src/zbot
tar tzf ~/Downloads/zbot_cameras_2026-11-02.tar.gz     # optional: list what's inside first
tar xzf ~/Downloads/zbot_cameras_2026-11-02.tar.gz     # extract everything into the repo

Running it from ~/zbot_ws/src/zbot matters, because the paths inside the tarball start there. The x means extract, z means it’s gzipped, and f names the file. It creates any folders that don’t exist yet. It replaces files that already exist;

============================
