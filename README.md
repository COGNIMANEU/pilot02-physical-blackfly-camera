
docker build -t pilot02-physical-blackfly-camera .

docker run -e LAUNCH_FILE=spinnaker_synchronized_camera_driver/launch/master_example.launch.py pilot02-physical-blackfly-camera


    
# Pilot02-Physical Blackfly Camera ROS 2 Driver



docker build -t pilot02-physical-blackfly-camera .

This repository contains the ROS 2 driver for **Pilot02-Physical Livox Lidar**. It includes all the necessary configurations, launch files, and a Docker setup to deploy the Livox Lidar driver in a containerized environment.

## Description

This project provides an easy-to-use setup for integrating Livox Lidar devices into ROS 2. The `livox_ros_driver2` node supports configuration via JSON and RViz files, and the system can be run inside a Docker container for ease of use.

The repository includes the following components:
- Configuration files for the Livox Lidar device
- Launch files for running the ROS 2 node
- Docker setup for easy containerization
- A docker-compose based test environment to verify the topics published by the Livox driver (lidar hardware required)

## Guidelines for build and test the component 

### 1. **Build the Main Docker Image:**

In this step, we build the Docker image using the provided `Dockerfile`. The image is named `pilot02-physical-blackfly-camera`.

```bash
docker build -t pilot02-physical-blackfly-camera .
```

Make sure the path to your configuration and launch files is correctly mapped to the Docker container.

### 2. **Run the ROS 2 Container:**

After building the Docker image, you can run the container using the following command:

```bash
docker run -e LAUNCH_FILE=your_custom_launch_file.py pilot02-physical-blackfly-camera
```

This will start the container and launch the ROS 2 node with the configured launch file.

### 3. **Build and Run the test automation:**

Test automation is integrated by docker-compose file:

Run: 
```bash
docker-compose up --build
```

In case lidar hardware is ready, you should see:
```python
TO BE COMPLETED!
```

In case of errors, you should see:
```python
TO BE COMPLETED!
```

## Example for cogniman pilot02

Build component: 
```bash
docker build  -t pilot02-physical-blackfly-camera .
```

Rebuild component (no cache):
```bash
docker build --no-cache -t pilot02-physical-blackfly-camera .
```

Run component : 
```bash
docker run --rm -it --device=/dev/video0:/dev/video0 -e CAMERAMODULE=spinnaker_synchronized_camera_driver -e LAUNCH_FILE=master_example.launch.py pilot02-physical-blackfly-camera 
```

docker run --rm -it -e CAMERAMODULE=spinnaker_synchronized_camera_driver -e LAUNCH_FILE=master_example.launch.py pilot02-physical-blackfly-camera 

docker run --rm -it --device=/dev/video0:/dev/video0 -e CAMERAMODULE=spinnaker_synchronized_camera_driver -e LAUNCH_FILE=master_example.launch.py pilot02-physical-blackfly-camera 


Build test: 
```bash
docker build -t pilot02-physical-blackfly-camera ./test
```

Rebuild test (no cache):
```bash
docker build --no-cache -t pilot02-physical-blackfly-camera ./test
```
Run test: 
```bash
docker run -it --rm pilot02-physical-blackfly-camera
```

## Contributing

Feel free to open issues or submit pull requests. Contributions are welcome!

## License

This project is licensed under the Apache 2 Licence, but includes MIT License and 33-Clause BSD License componentes - see the [LICENSE](LICENSE) file for details.