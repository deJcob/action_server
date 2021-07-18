# Action Server (and maybe client)

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


## Advanced tips and tricks

To test your server you want probably send some message. First check list of topics using
    rostopic list
    
Then open terminal and write:

    source devel/setup.bash
    rostopic pub /your_topic_name/goal your_msg

Due to the fact that your message is probably complicated use TAB key to get whole message structure and change parameters in the way you want. In this case this will look like below

    rostopic pub /FrontDocking/goal action_server/FrontDockingActionGoal "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
    goal_id:
    stamp:
        secs: 0
        nsecs: 0
    id: ''
    goal:
    order: 10" 

Also, if you want to check your feedback, goal, result or anything else published in system you can use rostopic echo /topic_name

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Helpful links

- http://wiki.ros.org/actionlib
- http://wiki.ros.org/actionlib/Tutorials
- http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
- http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29
- http://wiki.ros.org/actionlib_tutorials/Tutorials/Calling%20Action%20Server%20without%20Action%20Client

## License
[MIT](https://choosealicense.com/licenses/mit/)