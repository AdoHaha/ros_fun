up:
	docker-compose up

jupyter:
	docker container exec -it --user ubuntu ros_fun bash -i /home/ubuntu/run_jupyter.sh
