export ROBOT_IP="192.168.0.10$1"
rsync -r -l -p -t -v ../ fiborobotlab@${ROBOT_IP}:/home/fiborobotlab/dev_ws/src/fiborobotlab2
