ssh-keygen -t rsa
export ROBOT_IP="192.168.0.10$1"
ssh-copy-id -i ~/.ssh/id_rsa.pub fiborobotlab@ROBOT_IP