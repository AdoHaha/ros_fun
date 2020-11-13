up:
	docker-compose up

jupyter:
	docker container exec -it --user ubuntu ros_fun_ros2_1 bash -i /home/ubuntu/run_jupyter.sh
