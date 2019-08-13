xterm -geometry 80x10+2400+0 -e "/opt/ros/kinetic/bin/rosrun ii_go_get_it ggi_naigation.py" &
sleep 2s
xterm -geometry 80x10+2400+200 -e "/opt/ros/kinetic/bin/rosrun ii_go_get_it ggi_master.py" &
sleep 2s
