    
# Pilot02-Physical Blackfly Camera ROS 2 Driver

This repository contains the ROS 2 driver for **Pilot02-Physical Blackfly Camera**. It includes all the necessary configurations, launch files, and a Docker setup to deploy the Camera driver in a containerized environment.

## Description

This project provides an easy-to-use setup for integrating Blackfly Camera devices into ROS 2. The `cam_sync` node supports configuration, and the system can be run inside a Docker container for ease of use.

The repository includes the following components:
- Configuration files 
- Launch files for running the ROS 2 node
- Docker setup for easy containerization
- A docker-compose based test environment to verify the topics published by the driver (cameras hardware required)

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
docker run -e LAUNCH_FILE=your_custom_launch_file.py -e CAMERAMODULE=your_camera_module pilot02-physical-blackfly-camera
```

This will start the container and launch the ROS 2 node with the configured launch file.

### 3. **Build and Run the test automation:**

Test automation is integrated by docker-compose file:

Run: 
```bash
docker-compose up --build
```

In case camera hardware is ready (test search for two cameras), you should see:
```python
✅ Camera image received from cam0
✅ Camera image received from cam1
```

In case of errors, you should see:
```python
❌ No image received from cam0
❌ No image received from cam1
```

## Camera configuration

Include your camera serial number in the launch file:

```json
camera_list = {
    'cam0': '20435008',
    'cam1': '20415937',
}
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

Run component: 
```bash
docker run --rm -it -e CAMERAMODULE=spinnaker_synchronized_camera_driver -e LAUNCH_FILE=master_example.launch.py pilot02-physical-blackfly-camera 
```

Build test: 
```bash
docker build -t test-pilot02-physical-blackfly-camera ./test
```

Rebuild test (no cache):
```bash
docker build --no-cache -t test-pilot02-physical-blackfly-camera ./test
```
Run test: 
```bash
docker run -it --rm test-pilot02-physical-blackfly-camera
```

## Contributing

Feel free to open issues or submit pull requests. Contributions are welcome!

## License

This project is licensed under the Apache 2 Licence - see the [LICENSE](LICENSE) file for details.