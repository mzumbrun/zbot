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