razer_hydra
===============

ROS package for razer hydra motion controller. 
This is an unofficial driver which does NOT use the official Sixense SDK.

See documentation on the ros wiki: http://www.ros.org/wiki/razer_hydra

To find your connected Razer Hydra device, please run following commands in the terminal:

```bash
sudo cp config/99-hydra-indigo.rules /usr/lib/udev/rules.d/
```

Then, reactivate the rules via

```bash
udevadm control --reload-rules
```

After that, you may make the ros workspace with catkin_make with this repository included in the src folder.

Then, run following command in the terminal to connect Razer Hydra:

```bash
roslaunch razer_hydra hydra.launch
```

You attempt to echo the corresponding rostopics to check whether the device is validated.

TEST 
