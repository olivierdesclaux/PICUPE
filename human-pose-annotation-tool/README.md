# human-pose-annotation-tool

This work is derived from the repo: https://github.com/aimagelab/human-pose-annotation-tool


## Workflow

Load first RGB and depth image of video sequence in img folder. When calling function, default human pose appears on RGB image. Move markers around with the mouse, and click 'y' to confirm each position.
Confirm position by pressing enter. When finished, press enter to save the 2D HP. 
The script will then go fetch the depth data thanks to the registration and save the 3D data to the annotations file. 

You can specify input pose if you want to modify an existing set of markers. 

### TODO

- Consider registration: should we ask for pre-registered images or default images and a registration matrix? 

- Once registration is done, convert 2D pixel position to 3D distances in mm. 

- Modify --kpts_in argument to read 3D joints into 2D. 

- Change saving dict to a dict with each key being a marker and value is 3D position

- Save parameters into xml file for OpenSim scaling. 

