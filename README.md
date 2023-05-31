# augmanity_volumetric_detection
Volumetric detection on a collaborative cell

## Prerequisites

```
sudo apt-get install ros-noetic-octomap-* ros-noetic-octomap
```

## Run instructions

Change the initial parameters in `augmanity_volumetric_detection/scripts/cell_occupancy`, such as:

- `world_frame` - for the desired frame to measure octomap (please change it in `augmanity_volumetric_detection/launch/full_octomap.launch` as well)
- `threshold` - for the percentage of filled space in the desired volume
- `position` and `side_lengths` - for defining the volume

Also be sure to verify if all your sensors are on the `augmanity_volumetric_detection/launch/include/pc_relay.launch`


Then, please run:
`roslaunch augmanity_volumetric_detection cell_occupancy_detection.launch`
