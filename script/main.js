const app = Vue.createApp({

    el: "#app",
    data() {
        return {
            // ros connection
            ros: null,
            config: true,
            catresian: false,
            rosbridge_address: 'ws://192.168.1.82:9090/',
            connected: false,
            error: false,
            actuatorDone: true,
            serial_msg: "done",
            ArmRunning: true,
            // subscriber data
            cposition: {
                Joint1: 0.0,
                Joint2: 0.0,
                Joint3: 0.0,
                Joint4: 0.0,
                Joint5: 0.0,
                Joint6: 0.0,
            },
            cpositionxyz: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },

            j1: 0.00,
            j2: 0.00,
            j3: 0.00,
            j4: 0.00,
            j5: 0.00,
            j6: 0.00,
            delay: 0,
            title: '',
            history: [],
            // menu_title: 'Connection',
            // main_title: 'Main title, from Vue!!',
            keyValue: 'A',
            speed: 100,
            store: [],
            catTitle: '',
            actuatorName: '',
            actionName: '',

            actionList: [],
            blink: 1,
            buzz: 1,

            catList: [],
            ShowOperations: false,
            keyboardSpeed: 0.4,
            selectedlink: 'j6_Link',
            increasSize: 0.1,
            axisx: 0.0,
            axisy: 0.0,
            axisz: 0.0,


        }
    },
    methods: {



        connect: function () {
            // define ROSBridge connection object
            console.log(this.rosbridge_address)
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')


                localStorage.setItem("RUN", "");
                localStorage.setItem("ite", 0);
                this.todo()

            })
            this.ros.on('error', (error) => {
                this.error = true
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
                document.getElementById('divCamera').innerHTML = ''
            })
            this.showData()

        },
        showData: function () {

            try {
                this.catTitle = document.getElementById('catTitle1').value;
                if (localStorage.getItem(this.catTitle) === null) {

                    localStorage.setItem(this.catTitle, JSON.stringify(this.store));
                }
                else {

                    this.store = JSON.parse(localStorage.getItem(this.catTitle));
                }
            }
            catch (err) {
                console.log('eeeeeeeeeeeee')
            }


            if (localStorage.getItem('catlist') === null) {

                localStorage.setItem('catlist', JSON.stringify(this.catList));
            }
            else {
                this.catList = JSON.parse(localStorage.getItem('catlist'));
            }




        },
        disconnect: function () {
            this.ros.close()
        },



        ActuatorCommand: function (action) {
            // console.log("Action:")
            console.log(action)
            var cmdVel = new ROSLIB.Topic({
                ros: this.ros,
                name: action.actionCmd,
                messageType: "std_msgs/msg/String",
            });
            var msg = new ROSLIB.Message({
                data: action.action.toString()
            });
            // console.log(msg)
            // console.log(cmdVel)
            cmdVel.publish(msg);
        },


        Stop: function () {
            var serviceCall2 = new ROSLIB.Service({
                ros: this.ros,
                name: "/arm/Stop",
                serviceType: "std_srvs/srv/Trigger",
            });


            serviceCall2.callService(null, function (result2) {
                console.log('Result for service call on '
                    + serviceCall2.name
                    + ': '
                    + result2.success
                    + ' - '
                    + result2.message);
            });
        },
        // Stop: function () {
        //     var cmdVel = new ROSLIB.Topic({
        //         ros: this.ros,
        //         name: "/arm/stop",
        //         messageType: "std_msgs/msg/String",
        //     });
        //     var msg = new ROSLIB.Message({
        //         data: "stop"
        //     });

        //     cmdVel.publish(msg);
        // },

        addItem: function () {

            let item = {
                joint1: this.j1,
                joint2: this.j2,
                joint3: this.j3,
                joint4: this.j4,
                joint5: this.j5,
                joint6: this.j6
            }
            this.history.push(item)

            // this.$refs.table.scrollIntoView({ block: "end" });
        },


        Delete: function (index) {
            console.log(index)
            // this.store.splice(this.store.indexOf(index), 1);  
            this.store.splice(index, 1);
            localStorage.setItem(this.catTitle, JSON.stringify(this.store));
        },
        DeleteFromCatList: function (index) {
            // this.store.splice(this.store.indexOf(index), 1);  
            this.catList.splice(index, 1);
            localStorage.setItem("catlist", JSON.stringify(this.catList));
        },
        docycle: async function () {
            this.store = JSON.parse(localStorage.getItem(this.catTitle));

            let data = this.store;
            this.Operate(data)
        },
        doOperations: async function () {
            this.catList = JSON.parse(localStorage.getItem('catlist'));
            // console.log(this.catList[0])
            let run = []
            for (item in this.catList) {
                let data = JSON.parse(localStorage.getItem(this.catList[item].name));

                for (pos in data) {
                    run.push(data[pos])
                }
            }
            this.Operate(run)


        },


        Operate: async function (data) {
            // console.log(data)
            localStorage.setItem("RUN", JSON.stringify(data));
            localStorage.setItem("ite", 0);
            this.DO();


        },

        DO: async function () {
            let command = JSON.parse(localStorage.getItem("RUN"));
            let pos = localStorage.getItem("ite");
            if (command[pos] == null) {
                console.log("nothinggggggggggggggggggg")
                return
            };

            console.log(command[pos])
            while (command[pos].type.includes("Arm") == true) {
                // if (this.ArmRunning == false) {
                this.sendWithTarget(command[pos].value, command[pos].type)
                while (!this.reach(command[pos])) {
                    await this.sleep(500)
                    console.log('wait')
                }

                // while (this.ArmRunning == true) {
                //     await this.sleep(500)
                //     console.log('wait')
                // }

                // await this.sleep(data[pos].value.delay - 1 * 1000);
                // this.serial_msg = "working"       
                pos++;
                // this.sleep(500)
                // }

            }

            // else {



            if (command[pos].type == "LED") {
                let LEDaction = {
                    "action": command[pos].value.reapet,
                    "actionCmd": "/led/command"
                }
                this.serial_msg = "led"
                // msg = "led"
                this.ActuatorCommand(LEDaction)

            }

            else if (command[pos].type == "Buzzer") {
                let Buzzeraction = {
                    "action": command[pos].value.reapet,
                    "actionCmd": "/buzzer/command"
                }
                this.serial_msg = "buz"
                // msg = "buz"
                this.ActuatorCommand(Buzzeraction)

            }

            else if (command[pos].type == "Hand") {
                let handaction = {
                    "action": command[pos].value.action,
                    "actionCmd": "/hand/command"
                }
                let msg = ""
                if (command[pos].value.action == "pickup") {
                    this.serial_msg = "up"
                    msg = "up";
                }
                else {
                    this.serial_msg = "off"
                    msg = "off";
                }
                this.ActuatorCommand(handaction);
                // this.sleep(2000);
                // console.log("exit")

            }


            // await this.sleep(1000)
            // console.log("Next---------------------")




            // }
            pos++;
            localStorage.setItem("ite", pos);
            console.log(pos)
        },


        ARMState: function () {
            this.ArmRunning = true
            var listener = new ROSLIB.Topic({
                ros: this.ros,
                name: "/arm/run",
                messageType: "std_msgs/msg/String"
            });
            listener.subscribe((message) => {
                // console.log(message.data)
                if (message.data == "1") {
                    this.ArmRunning = true
                }
                else {
                    this.ArmRunning = false;

                }
                // console.log(message)
                // this.actuatorDone=  message.data.includes(this.serial_msg)
                // console.log(this.actuatorDone)
            });

            return this.ArmRunning;
            // return item
        },
        // ActuatorState: function (msg) {
        //     var done = false;
        //     var listener = new ROSLIB.Topic({
        //         ros: this.ros,
        //         name: "/actuator/state",
        //         messageType: "std_msgs/msg/String"
        //     });
        //     // let back = false;
        //     listener.subscribe((message) => {

        //         if (message.data.includes(msg)) {
        //             console.log(message.data.trim(), "*********", msg)
        //             this.actuatorDone = true;
        //             this.serial_msg = "done"
        //             done = true
        //         }
        //         else {
        //             this.actuatorDone = false;
        //             done = false
        //         }
        //         // this.actuatorDone = message.data.includes(msg)
        //     });
        //     // console.log(this.actuatorDone)
        //     // console.log(back)
        //     // // return this.actuatorDone;
        //     return done;

        //     // return item
        // },

        ActuatorState2: function () {
            var listener = new ROSLIB.Topic({
                ros: this.ros,
                name: "/actuator/state",
                messageType: "std_msgs/msg/String"
            });
            // let back = false;
            listener.subscribe((message) => {

                if (message.data.includes(this.serial_msg)) {
                    // console.log(message.data.trim(), "*********", this.serial_msg)
                    console.log("uppppppppp offffffffff")
                    this.DO()


                }
                else if (message.data.includes("A") || message.data.includes("B")) {
                    console.log("AAAA BBBBB")
                    this.DO()
                }
                else {
                    this.actuatorDone = false;
                }
            });
            // console.log(this.actuatorDone)

            // // return this.actuatorDone;

            // return item
        },

        reach: function (target) {
            // console.log(Math.round(this.cposition.Joint1))
            // console.log(target.value)
            // console.log(this.cposition)

            if (Math.round(this.cposition.Joint1) != target.value.joint1)
                return false;
            if (Math.round(this.cposition.Joint2) != target.value.joint2)
                return false;
            if (Math.round(this.cposition.Joint3) != target.value.joint3)
                return false;
            if (Math.round(this.cposition.Joint4) != target.value.joint4)
                return false;
            if (Math.round(this.cposition.Joint5) != target.value.joint5)
                return false;
            if (Math.round(this.cposition.Joint6) != target.value.joint6)
                return false;

            return true;
        },
        sleep: function (milliseconds) {
            // return new Promise((resolve) => setTimeout(resolve, milliseconds));
            return new Promise(resolve => setTimeout(resolve, milliseconds));
        },

        sleepNow: function (delay) {
            new Promise((resolve) => setTimeout(resolve, delay))
        },
        currentPosition: function () {
            this.j1 = Math.round(this.cposition.Joint1)
            this.j2 = Math.round(this.cposition.Joint2)
            this.j3 = Math.round(this.cposition.Joint3)
            this.j4 = Math.round(this.cposition.Joint4)
            this.j5 = Math.round(this.cposition.Joint5)
            this.j6 = Math.round(this.cposition.Joint6)

            this.axisx = Math.round(this.cpositionxyz.x)
            this.axisy = Math.round(this.cpositionxyz.y)
            this.axisz = Math.round(this.cpositionxyz.z)
        },

        showPosition: function () {
            var serviceCall = new ROSLIB.Service({
                ros: this.ros,
                name: "/arm/GetState",
                serviceType: "arm_msgs/srv/GetState",
                header: {

                    // rclnodejs doesn't automatically set the timestamp
                    stamp: {
                        sec: Math.floor((new Date()).getTime() / 1000),
                        // nanosec: window.performance.now() ???,
                    },

                    frame_id: "arm_link"
                },
            });


            // console.log('call')
            serviceCall.callService('', (result2) => {
                // console.log('Result for service call on '
                //     + serviceCall2.name
                //     + ': '
                //     + result2.success);
                // console.log('called')

                let pos = result2.joint_pos_deg;
                // let txt = "";
                // // let i = 0;
                // for (let x in poses) {
                //     p = ((poses[x] * 180) / Math.PI) % 360;
                //     // p =poses[x]
                //     // if (p > 180) {
                //     //     p = p - 360;
                //     // }
                //     poses[x] = p;
                // }

                this.cposition.Joint1 = pos[0]
                this.cposition.Joint2 = pos[1]
                this.cposition.Joint3 = pos[2]
                this.cposition.Joint4 = pos[3]
                this.cposition.Joint5 = pos[4]
                this.cposition.Joint6 = pos[5]

                this.cpositionxyz = {
                    x: result2.coordinates.x,
                    y: result2.coordinates.y,
                    z: result2.coordinates.z
                }
            });

            // console.log(this.cposition)

            // return item
        },


        showPosition2: function () {
            var serviceCall2 = new ROSLIB.Service({
                ros: this.ros,
                name: "/get_planning_scene",
                serviceType: "moveit_msgs/srv/GetPlanningScence",
            });


            serviceCall2.callService(null, (result2) => {
                console.log('Result for service call on '
                    + serviceCall2.name
                );
                var pos = result2.scene.robot_state.joint_state.position
                // let item = {
                //     Joint1:pos[0],
                //     Joint2:pos[1],
                //     Joint3:pos[2],
                //     Joint4:pos[3],
                //     Joint5:pos[4],
                //     Joint6:pos[5],
                // }

                this.cposition.Joint1 = pos[0]
                this.cposition.Joint2 = pos[1]
                this.cposition.Joint3 = pos[2]
                this.cposition.Joint4 = pos[3]
                this.cposition.Joint5 = pos[4]
                this.cposition.Joint6 = pos[5]
                // this.cposition = item;
                // console.log(this.cposition)
            });
        },

        // showPosition: function () {
        //     var listener = new ROSLIB.Topic({
        //         ros: this.ros,
        //         name: "/joint_states",
        //         messageType: "sensor_msgs/msg/JointState",
        //     });
        //     listener.subscribe((message) => {
        //         //    this.cposition= listener.subscribe(function (message) {

        //         let poses = message.position;
        //         console.log(poses);
        //         let txt = "";
        //         // let i = 0;
        //         for (let x in poses) {
        //             p = ((poses[x] * 180) / Math.PI) % 360;
        //             // p =poses[x]
        //             // if (p > 180) {
        //             //     p = p - 360;
        //             // }
        //             poses[x] = p;
        //         }

        //         listener.unsubscribe();
        //         let item = {
        //             Joint1: poses[0],
        //             Joint2: poses[1],
        //             Joint3: poses[2],
        //             Joint4: poses[3],
        //             Joint5: poses[4],
        //             Joint6: poses[5],
        //         }

        //         this.cposition = item;
        //     });

        //     // return item
        // },
        send: function () {
            if (this.catresian == false) {
                var serviceCall = new ROSLIB.Service({
                    ros: this.ros,
                    name: "/arm/JointSpaceGoal",
                    serviceType: "arm_msgs/srv/JointSpaceGoal",
                    header: {

                        // rclnodejs doesn't automatically set the timestamp
                        stamp: {
                            sec: Math.floor((new Date()).getTime() / 1000),
                            // nanosec: window.performance.now() ???,
                        },

                        frame_id: "arm_link"
                    },
                });

                var request = new ROSLIB.ServiceRequest({
                    // name: ["j1", "j2", "j3", "j4", "j5", "j6"],
                    joint_pos_deg: [this.j1, this.j2, this.j3, this.j4, this.j5, this.j6],
                    speed: parseInt(this.speed),
                });
                console.log(request)


                serviceCall.callService(request, function (result) {
                    console.log('Result for service call on '
                        + serviceCall.name
                        + ': '
                        + result.valid);
                    // if (result.valid == true)
                    //     this.Execute()
                    // return  result.valid;

                });

                this.Execute();
            }
            else {
                values_Cartesian_Jogging = { x: this.axisx * 1, y: this.axisy * 1, z: this.axisz * 1 }
                send_Cartesian_Jog(values_Cartesian_Jogging, "arm_link")

            }
            this.addItem()

        },

        sendWithTarget: function (target, type) {

            if (type == "Arm Angles") {
                var serviceCall = new ROSLIB.Service({
                    ros: this.ros,
                    name: "/arm/JointSpaceGoal",
                    serviceType: "arm_msgs/srv/JointSpaceGoal",
                });


                rj1 = target.joint1 * 1;
                rj2 = target.joint2 * 1;
                rj3 = target.joint3 * 1;
                rj4 = target.joint4 * 1;
                rj5 = target.joint5 * 1;
                rj6 = target.joint6 * 1;


                var request = new ROSLIB.ServiceRequest({
                    // name: ["j1", "j2", "j3", "j4", "j5", "j6"],
                    joint_pos_deg: [rj1, rj2, rj3, rj4, rj5, rj6],
                    speed: parseInt(this.speed),
                });
                console.log(request)


                serviceCall.callService(request, function (result) {
                    console.log('Result for service call on '
                        + serviceCall.name
                        + ': '
                        + result);
                });
            }
            else {
                values_Cartesian_Jogging = { x: target.axisx * 1, y: target.axisy * 1, z: target.axisz * 1 }
                send_Cartesian_Jog(values_Cartesian_Jogging, "arm_link")
            }

        },


        Execute: function () {
            var serviceCall2 = new ROSLIB.Service({
                ros: this.ros,
                name: "/arm/Execute",
                serviceType: "std_srvs/srv/Trigger",
            });


            serviceCall2.callService(null, function (result2) {
                console.log('Result for service call on '
                    + serviceCall2.name
                    + ': '
                    + result2.success
                    + ' - '
                    + result2.message);
            });

        },
        clearHistory: function () {
            this.history = '';
        },
        clearPositions: function () {
            localStorage.setItem('', JSON.stringify([]));
            this.store = [];
        },


        Jogging_Command: function (value) {
            var serviceCall2 = new ROSLIB.Service({
                ros: this.ros,
                name: "/servo_node/switch_command_type",
                serviceType: "moveit_msgs/srv/ServoCommandType",
            });
            var request = new ROSLIB.ServiceRequest({
                command_type: value
            });
            serviceCall2.callService(request, function (result2) {
                console.log('Result for service call on '
                    + serviceCall2.name
                    + ': '
                    + result2.success);
            });

        },

        sendKey_Jogging: function (values) {
            // this.Jogging_Command()
            var cmdVel = new ROSLIB.Topic({
                ros: this.ros,
                name: "/servo_node/delta_joint_cmds",
                messageType: "control_msgs/msg/JointJog",

            });

            var msg = new ROSLIB.Message({
                header: {

                    // rclnodejs doesn't automatically set the timestamp
                    stamp: {
                        sec: Math.floor((new Date()).getTime() / 1000),
                        // nanosec: window.performance.now() ???,
                    },

                    frame_id: "arm_link"
                },
                joint_names: ["j1", "j2", "j3", "j4", "j5", "j6"],
                displacements: [],
                velocities: values,
                duration: 0.0
            });
            console.log(msg)
            cmdVel.publish(msg);

        },

        send_Cartesian_Jog: function (values, frame_id_value) {
            var cmdVel = new ROSLIB.Topic({
                ros: this.ros,
                name: "/servo_node/delta_twist_cmds",
                messageType: "geometry_msgs/msg/TwistStamped",

            });

            var msg = new ROSLIB.Message({
                header: {

                    // rclnodejs doesn't automatically set the timestamp
                    stamp: {
                        sec: Math.floor((new Date()).getTime() / 1000),
                        // nanosec: window.performance.now() ???,
                    },

                    frame_id: frame_id_value,
                },
                twist: {
                    linear: values,
                    angular: { x: 0.0, y: 0.0, z: 0.0 }
                }
            });
            console.log(msg)
            cmdVel.publish(msg);

        },


        moveByKey() {
            let values_Jogging = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            let clockwise = this.keyboardSpeed * 1
            let counterclockwise = this.keyboardSpeed * -1
            values_Cartesian_Jogging = { x: 0.0, y: 0.0, z: 0.0 }

            console.log(this.keyValue)

            switch (this.keyValue.toLowerCase()) {
                case 'z':
                    // this.j1--;
                    values_Jogging[0] = counterclockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;
                case 'x':
                    // this.j1++;
                    values_Jogging[0] = clockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;

                case 'a':
                    // this.j2--;
                    values_Jogging[1] = counterclockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;
                case 's':
                    // this.j2++;
                    values_Jogging[1] = clockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;

                case 'q':
                    // this.j3--;
                    values_Jogging[2] = counterclockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;
                case 'w':
                    // this.j3++;
                    values_Jogging[2] = clockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;

                case 'n':
                    // this.j4--;
                    values_Jogging[3] = counterclockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;
                case 'm':
                    // this.j4++;
                    values_Jogging[3] = clockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;

                case 'k':
                    // this.j5--;
                    values_Jogging[4] = counterclockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;
                case 'l':
                    // this.j5++;
                    values_Jogging[4] = clockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;

                case 'o':
                    // this.j6--;
                    values_Jogging[5] = counterclockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;
                case 'p':
                    // this.j6++;
                    values_Jogging[5] = clockwise;
                    this.sendKey_Jogging(values_Jogging);
                    break;



                case 'arrowup':
                    // this.j6++;                    
                    values_Cartesian_Jogging.x = this.increasSize * 1;
                    this.send_Cartesian_Jog(values_Cartesian_Jogging, this.selectedlink);
                    break;

                case 'arrowdown':
                    // this.j6++;                    
                    values_Cartesian_Jogging.x = this.increasSize * -1
                    this.send_Cartesian_Jog(values_Cartesian_Jogging, this.selectedlink);
                    break;

                case 'arrowright':
                    // this.j6++;                    
                    values_Cartesian_Jogging.y = this.increasSize * 1
                    this.send_Cartesian_Jog(values_Cartesian_Jogging, this.selectedlink);
                    break;

                case 'arrowleft':
                    // this.j6++;                    
                    values_Cartesian_Jogging.y = this.increasSize * -1
                    this.send_Cartesian_Jog(values_Cartesian_Jogging, this.selectedlink);
                    break;


                case '=':
                case '+':
                    // this.j6++;                    
                    values_Cartesian_Jogging.z = this.increasSize * 1
                    this.send_Cartesian_Jog(values_Cartesian_Jogging, this.selectedlink);
                    break;

                case '-':
                    // this.j6++;                    
                    values_Cartesian_Jogging.z = this.increasSize * -1
                    this.send_Cartesian_Jog(values_Cartesian_Jogging, this.selectedlink);
                    break;


                case 'Enter', 'Escape', ' ':
                    console.log('Stop');
                    this.Stop()
                    break;
            }
        },

        getKey(evt) {
            this.keyValue = evt.key
            console.log(evt.key)
            this.moveByKey()
        },

        addPosition: function () {


            if (this.catresian == false) {
                let item = {
                    type: "Arm Angles",
                    value: {
                        joint1: this.j1,
                        joint2: this.j2,
                        joint3: this.j3,
                        joint4: this.j4,
                        joint5: this.j5,
                        joint6: this.j6,
                        speed: this.speed,
                        delay: this.delay,
                        title: this.title,
                    },
                }
                // let positions = localStorage.getItem("positions");
                // positions.addItem(item);
                this.store.push(item);
            }
            else {
                let item = {
                    type: "Arm Coordinates",
                    value: {
                        axisx: this.axisx * 1,
                        axisy: this.axisy * 1,
                        axisz: this.axisz * 1,

                        speed: this.speed,

                        title: this.title,
                    },
                }
                this.store.push(item);
            }
            localStorage.setItem(this.catTitle, JSON.stringify(this.store));
        },

        addBlink: function () {
            let item = {
                type: "LED",
                value: {
                    reapet: this.blink,
                }
            }
            // let positions = localStorage.getItem("positions");
            // positions.addItem(item);
            this.store.push(item)
            localStorage.setItem(this.catTitle, JSON.stringify(this.store));

        },
        addBuzz: function () {
            let item = {
                type: "Buzzer",
                value: {
                    reapet: this.buzz,
                }
            }
            // let positions = localStorage.getItem("positions");
            // positions.addItem(item);
            this.store.push(item)
            localStorage.setItem(this.catTitle, JSON.stringify(this.store));


        },
        addHandAction: function () {
            let item = {
                type: "Hand",
                value: {
                    actuator: this.actuatorName,
                    action: this.actionName.actionCmd,
                    actionCmd: "/hand/command",
                }
            }
            // let positions = localStorage.getItem("positions");
            // positions.addItem(item);
            this.store.push(item)
            localStorage.setItem(this.catTitle, JSON.stringify(this.store));
            this.actionName = ''

        },
        AddToCatList: function () {
            this.catTitle = document.getElementById('catTitle1').value;

            let item = {
                name: this.catTitle,
            }
            console.log(this.catTitle)
            console.log(item)
            this.catList.push(item)
            localStorage.setItem('catlist', JSON.stringify(this.catList));
        },

        saveWorkspace: function () {
            localStorage.setItem(this.catTitle, JSON.stringify(this.store));

            saveWorkspace()
        },
        modal: function (name) {

            this.store = JSON.parse(localStorage.getItem(name));
        },




        todo: function () {
            this.intervalid1 = setInterval(() => {
                this.showPosition();
                // this.sss();
                // // this.ARMState();
                // this.ActuatorState2();

            }, 1000);

        },
    },

    mounted() {
        // page is ready

        console.log('page is ready!');
        // Hand Menu
        this.actionList = actuators;
        this.showData()

    },
})


app.mount('#app')