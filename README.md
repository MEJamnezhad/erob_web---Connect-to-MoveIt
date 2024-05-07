
***Install and Run Web Bridge (Web Socket) on Linux***
## 0- Prepare
    - sudo su 
    - source /opt/ros/iron/setup.bash
## 1- install rosbridge_server on iron 
    sudo apt-get install ros-iron-rosbridge-server

## 2- Run rosbridge_server and it made a web socket on 9090 port

    - ros2 launch rosbridge_server rosbridge_websocket_launch.xml

## 3-1- Run erob_pkg
    - sodo su ros2 run erob_pkg erob_node

## 3-2- After that, Open web page
    some main functions on javascript:
        - var ros = new ROSLIB.Ros();

        -  ros.on(['error'], function(error) {
                ...
            });
        
        -  ros.on(['connection'], function(error) {
                ...
            });
        
        -  ros.on(['close'], function(error) {
                ...
            });

        - ros.connect('ws://127.0.0.1:9090/');

        - Publish a Topic 
            var cmdVel = new ROSLIB.Topic({
                ros : ros,
                name: '/arm/command2',  
                messageType: 'sensor_msgs/msg/JointState'
            });

            var JointState = new ROSLIB.Message({
                position: [0.3,0.3,0.3,0.3,0.3,0.3],
                        velocity: [10,10,10,10,10,10],
                        effort:[]
            });
            cmdVel.publish(JointState);

        - Subscribing to a Topic

            var listener = new ROSLIB.Topic({
                ros : ros,
                name: '/arm/state2',
                messageType: 'sensor_msgs/msg/JointState'
            });

            listener.subscribe(function(message) {
                console.log('Received message on ' + listener.name + ': ' + message.position);
                listener.unsubscribe();
            });