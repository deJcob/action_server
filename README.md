# Action Server

Action Server is a tool for research connected with vc200_controller.

Used with ROS melodic/noetic. 

## Usage

Firstly, make sure that your python path is correct 

    #! /usr/bin/env python3

or 

    #! /usr/bin/env python2.7

If not, change it in server files in /src path. Alse remebmer about execut permission 

    chmod +x FrontDockingActionServer.py

To run server you have to run everything normally

    roscore

Then run server by

    source devel/setup.bash
    rosrun action_server FrontDockingActionServer.py


## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Helpful links

    http://wiki.ros.org/actionlib
    http://wiki.ros.org/actionlib/Tutorials
    http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
    http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29
    http://wiki.ros.org/actionlib_tutorials/Tutorials/Calling%20Action%20Server%20without%20Action%20Client

## License
[MIT](https://choosealicense.com/licenses/mit/)