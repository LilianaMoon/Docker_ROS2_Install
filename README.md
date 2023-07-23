# The task revolves around ROS, Linux, and Docker.
This project, named "Docker_ROS2_Install," provides a simple Dockerfile for setting up a Docker container with Ubuntu 20.04 and installing ROS 2 Foxy base (No GUI tools). The container includes a basic ROS node for continuous monitoring of CPU utilization, which writes the information to a log file and displays it in the console. Additionally, there is a shell script for generating CPU load.

# Installation
To install ROS2 Foxy and set up the Docker container, execute the command docker-compose up inside this project. The installation process will automatically configure ROS2 Foxy and set up the ROS node, which writes CPU usage data to a log file named "cpu_load.log" every 5 seconds.

Please note that the installation process may take approximately 4 minutes to complete.

ATTENTION! The installation has been done without desktop support as this is a test version.

After the container named "test_task" finishes loading, it will start automatically. To view the logs obtained from the "cpu_log" in the container, open another terminal and run the command ./logs_from_container.sh.

If desired, you can run ./generateLoad.sh in a new terminal to put sufficient load on the CPU and test your pipeline. The program indicates that it should not exceed 80% CPU usage, and you can interrupt the command to stop the processes.
