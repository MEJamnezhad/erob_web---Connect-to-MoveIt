from gc import garbage
import math
from pickle import TRUE
from turtle import position
from matplotlib.dates import date_ticker_factory
from numpy import angle, mat
from sympy import false, true
import serial
import rclpy
from rclpy.node import Node
import struct
import pysoem
# import time
from time import sleep

# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from sensor_msgs.msg import JointState

master = pysoem.Master()
# master.open(r"\Device\NPF_{7FADE00F-793A-40F6-B97C-5DB7FEE9EADA}") 
master.open("eno1") 
# master.open("eth0") 

number_of_slaves =3

brake_Engage = 0x0300
brake_Disengage = 0x0301
brake_state = brake_Disengage

# active_joint=[]
# ----------------------------------
def getnbit(value, index,bits):
    """ Return n bits from index
    Parameters
    ----------
        number: int
            the value that need 
        index: int
            start bit
        bits: int
            number of bits
        

    Returns
    -------
        binary digit with (n)bits from index 
    """
    format = pow(2,bits) - 1
    return (value & (format << (index * bits))) >> (index * bits)
# ----------------------------------
def upload(device,deviceNo,code,offset=0,format='I',message=''):
    return struct.unpack(format, device.sdo_read(code, offset))[0]   
# ----------------------------------
def download(device,deviceNo,code,offset=0,format='I',message='',value=0):
    try:
        device.sdo_write(code, offset, struct.pack(format, value))
    except:
        None
# ----------------------------------
def changeTo_OP_Ethercat(slave, pos):
    """ Change slave state to Operation state on EtherCat

    Parameters
    ----------
        salve: master.salve
            Joint
    """    
    # print('Start to change state to OP')
    # SAFEOP && ERROR
    while slave.state != pysoem.OP_STATE:
        if slave.state == (pysoem.SAFEOP_STATE + pysoem.STATE_ERROR):
            # print('ERROR : Slave {} is in SAFE_OP + ERROR, attempting to acknowledge...'.format(pos))
            slave.state = pysoem.SAFEOP_STATE + pysoem.STATE_ACK
            slave.write_state()
        # SAFEOP_STATE
        elif slave.state == pysoem.SAFEOP_STATE:
            # print('WARNING : Slave {} is in SAFE_OP, trying to change to OPERATIONAL...'.format(pos))
            slave.state = pysoem.OP_STATE
            slave.write_state()
                
        # NONE_STATE
        elif slave.state > pysoem.NONE_STATE:
            if slave.reconfig():
                slave.is_lost = False
                # print('MESSAGE : Slave {} reconfigured...'.format(pos))
        # Check if slave is lost
        elif not slave.is_lost:
            slave.state_check(pysoem.OP_STATE)
            if slave.state == pysoem.NONE_STATE:
                slave.is_lost = True
                # print('ERROR : Slave {} lost...'.format(pos))
        # If lost, trying to recover
        if slave.is_lost:
            if slave.state == pysoem.NONE_STATE:
                if slave.recover():
                    # Recovery successful
                    slave.is_lost = False
                    # print('MESSAGE : Slave {} recovered...'.format(pos))
            else:
                # ??
                slave.is_lost = False
                # print('MESSAGE : Slave {} found...'.format(pos))
    # if slave.state == pysoem.OP_STATE:
    #     print('OK : Slave {} is in OP_STATE ...'.format(pos))
        
        # sleep(0.1)
    
    master.send_processdata()
    master.receive_processdata(timeout=2000)
    # sleep(0.02)
#-----------------------------------------------------
def shutdown(device):                      
    """ Change slave state to Init state

    Parameters
    ----------
        salve: master.salve
            Joint
        idx: int            
    """  
    device.state =pysoem.INIT_STATE
    device.write_state()
# --------------------------------
def changeTo_OP_Enable(device,idx):
    """ Change slave state to Enable operation

    Parameters
    ----------
        salve: master.salve
            Joint
        idx: int            
    """  
    result = upload(device,idx,0x6041,0,'H',message='Check state') 
    # print(getnbit(result,0,4))
    # if getnbit(result,0,4) == 7:
    #     print(f'Device {idx} Is Enable')
    #     return

    # download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
    
    # while result is not Switch on disabled: wait
    check =True
    while check:
        
        errorcode = upload(device,idx,0x603f,0,'H',message='Error Code')
        if  errorcode > 0:
            if idx<3:
                device_VelocityConfig(device,idx)
            else:
                small_device_VelocityConfig(device,idx)

            download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
            # download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
            download(device,idx,0x60FF,0,value=0,message='Target velocity set 0') 
            pos = upload(device,idx,0x6064,0,'I',message='The current position')
            download(device,idx,0x607A,0,value=pos,message='Change The target position')


        result = upload(device,idx,0x6041,0,'H',message='Check state2')  
        while getnbit(result,0,4) == 0b1000: 
            download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
            download(device,idx,0x60FF,0,value=0,message='Target velocity set 0') 

            # sleep(0.05)
            master.send_processdata()
            master.receive_processdata(2000)        
            # sleep(0.05)
            result = upload(device,idx,0x6041,0,'H',message='Check')

        # result = upload(device,idx,0x6041,0,'H',message='Check state2')  
        # print(getnbit(result,0,4))
        if getnbit(result,0,4) == 7:
#            print(f'Device {idx} Is Enable')
            return 
        val = getnbit(result,0,7) 
        # print(f'{val:06b}')    
        if (val == 0b0010000) or (val == 0b0110000):            
            continue
         
        if (val == 0b1010000) or (val == 0b1000000):
           download(device,idx,0x60FF,0,value=0,message='Target velocity set 0')  
           download(device,idx,0x6040,0,value=0x0006,message='Set ready to switch on')
        #    sleep(0.05)

     
        if (val == 0b0110001) or (val == 0b0100001): 
            download(device,idx,0x60FF,0,value=0,message='Target velocity set 0')  
            download(device,idx,0x6040,0,value=0x0007,message='Set switch on')
            # sleep(0.05)        
        
        if (val == 0b0110011) or (val == 0b0100011):  
            download(device,idx,0x60FF,0,value=0,message='Target velocity set 0')  
            download(device,idx,0x6040,0,value=0x000f,message='Set Enable operation')
            # sleep(0.05)    
# ----------------------------------
def changeTo_OP_Master(master):
    """ Change Master state to Operation state

    Parameters
    ----------
        master: pysoem.Master
            Master in EtherCat
    """  
    # wait 50 ms for all slaves to reach SAFE_OP state
    if master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
        master.read_state()
        for slave in master.slaves:
            if not slave.state == pysoem.SAFEOP_STATE:
                print('{} did not reach SAFEOP state'.format(slave.name))
                print('al status code {} ({})'.format(hex(slave.al_status),
                                                        pysoem.al_status_code_to_string(slave.al_status)))
        Exception('not all slaves reached SAFEOP state')

    master.state = pysoem.OP_STATE
    master.write_state()

    master.state_check(pysoem.OP_STATE, 500000)
    if master.state != pysoem.OP_STATE:
        master.read_state()
        for slave in master.slaves:
            if not slave.state == pysoem.OP_STATE:
                print('{} did not reach OP state'.format(slave.name))
                print('al status code {} ({})'.format(hex(slave.al_status),
                                                        pysoem.al_status_code_to_string(slave.al_status)))
        print('not all slaves reached OP state')

# ----------------------------------
def device_PDO_setup(device):
    """ Config PDO

    Parameters
    ----------
        device: Master.slave
            Joint
    """      
    # Select rx PDOs.
    rx_map_obj = [
        0x160F,
        0x161C,
        0x161D
    ]
    rx_map_obj_bytes = struct.pack(
        "Bx" + "".join(["H" for _ in range(len(rx_map_obj))]), len(rx_map_obj), *rx_map_obj)
    device.sdo_write(index=0x1C12, subindex=0, data=rx_map_obj_bytes, ca=True)


    # map_1c12_bytes = struct.pack('BxH', 1, 0x1600)
    # device.sdo_write(0x1c12, 0, map_1c12_bytes, True)

    map_1c13_bytes = struct.pack('BxH', 1, 0x1A00)
    # map_1c13_bytes = struct.pack('BxH', 1, 0x1A00)
    device.sdo_write(0x1c13, 0, map_1c13_bytes, True)
    
# ----------------------------------
def device_VelocityConfig(device,idx):
    """ Config Velocity and etc.

    Parameters
    ----------
        device: Master.slave
            Joint
        idx: int 
    """  
    download(device,idx,0x607F,0,value=145927,message='Set: Max profile velocity')#  145927 
    download(device,idx,0x6080,0,value=145927,message='Set: Max speed')    # 145927 
    download(device,idx,0x6081,0,value=60000,message='Set: Profile velocity') # 45000
    # download(device,idx,0x60FF,0,value=10,message='Set: target velocity')


    # download(device,idx,0x60C5,0,value=1400,message='Set: Max acceleration') #   486423          
    # download(device,idx,0x60C6,0,value=2800,message='Set: Max deceleration')  # 486423
    
    # download(device,idx,0x6083,0,value=1400,message='Set: Profile acceleration') # 291854
    # download(device,idx,0x6084,0,value=2800,message='Set: Profile deceleration') # 291854

    download(device,idx,0x6065,0,value=100000,message='Set: pos following window ')
    download(device,idx,0x6066,0,value=10000,message='Set: pos following window timeout')
    
    # download(device,idx,0x6068,0,value=10,message='Set: Position window time')
    download(device,idx,0x3b60,0,value=100000,message='Set: Maximum ')

# ----------------------------------
def small_device_VelocityConfig(device,idx):
    """ Config Velocity and etc.

    Parameters
    ----------
        device: Master.slave
            Joint
        idx: int 
    """  
    download(device,idx,0x607F,0,value=262144,message='Set: Max profile velocity')#  145927 
    download(device,idx,0x6080,0,value=262144,message='Set: Max speed')    # 145927 
    download(device,idx,0x6081,0,value=60000,message='Set: Profile velocity') # 45000
    # download(device,idx,0x60FF,0,value=10,message='Set: target velocity')


    # download(device,idx,0x60C5,0,value=1400,message='Set: Max acceleration') #   486423          
    # download(device,idx,0x60C6,0,value=2800,message='Set: Max deceleration')  # 486423
    
    # download(device,idx,0x6083,0,value=1400,message='Set: Profile acceleration') # 291854
    # download(device,idx,0x6084,0,value=2800,message='Set: Profile deceleration') # 291854

    download(device,idx,0x6065,0,value=100000,message='Set: pos following window ')
    download(device,idx,0x6066,0,value=10000,message='Set: pos following window timeout')
    
    # download(device,idx,0x6068,0,value=10,message='Set: Position window time')
    download(device,idx,0x3b60,0,value=100000,message='Set: Maximum ')



def move(idx,targetdegree,speed):
    device = master.slaves[idx]
    download(device,idx,0x6060,0,value=1,message='Set: PP mode')
    
    # pos = upload(device,idx,0x6064,0,'i',message='The current position')
    # print(f'{idx}: {pos} --  {postodegree(pos)} ')
    enc= 524288.0
    if targetdegree == 0:
        target =0
    else:
        target= int( targetdegree * (enc/360.0))


    # print(f'{target} --  {targetdegree} ')
    download(device,idx,0x607A,0,value=target,format='i', message='Change The target position')
    speed = int(speed * 50)
    download(device,idx,0x6081,0,value=speed,message='Set: Profile velocity') # 45000
    
    download(device,idx,0x6083,0,value=50000,message='Set: Profile acceleration') # 291854
    
    download(device,idx,0x6084,0,value=50000,message='Set: Profile deceleration') # 291854
    
    
    # download(device,idx,0x6040,0,value=0x0006,message='Set ready to switch on')
    # download(device,idx,0x6040,0,value=0x0007,message='Set ready to switch on')
    # download(device,idx,0x6040,0,value=0x000f,message='Set ready to switch on')
    changeTo_OP_Enable(device,idx)
    # sleep(1)
    download(device,idx,0x6040,0,value=0x001f,message='Set Command trigger')
    download(device,idx,0x6040,0,value=0x000f,message='Set ready to switch on')
    download(device,idx,0x6040,0,value=0x001f,message='Set Command trigger')

    # result = upload(device,idx,0x6041,0,'H',message='Check state2') 
    # while(result>> 10) % 2 == 0:
    #     sleep(0.1)
    #     result = upload(device,idx,0x6041,0,'H',message='Check state2') 
    #     # print(result)
    
    # print(f'Joint {idx} reached {targetdegree} degree')



def mainapp():
    try:
        number_of_slaves =master.config_init()

        if number_of_slaves > 0:
            print(f'Found {number_of_slaves} slaves')                        
            print('Master state: ',hex(master.read_state()))
            
            print('=================================================')
            
            master.config_map()

            changeTo_OP_Master(master)  

            for i,device in enumerate(master.slaves):
                changeTo_OP_Ethercat(device,i)    



            for i,device in enumerate(master.slaves):
                changeTo_OP_Enable(device,i)   


                                    
            # download(master.slaves[0],0,0x2242,0,value=1,message='Reset Encoder')

            print('=================================================')
            
            return number_of_slaves
        else:
            print('no device found')
            return 0

        return master

        
    except KeyboardInterrupt:
        # ctrl-C abort handling
        for i,device in enumerate(master.slaves):
            shutdown(device)   
        master.close()
        print('stopped')

#---------------------------------------
def releasebrake(device,idx):
    download(device,idx,0x6402,0,value=1,message='Set: Release Brake')

#---------------------------------------

def releasebrake(idx):
    device =master.slaves[idx]
    download(device,idx,0x6402,0,value=1,message='Set: Release Brake')

#--------------------------------------
        
def postodegree(pos):
    """ Convert position of encoder to degree

    Parameters
    ----------
        pos: int 
    """     
    # pos can be negetive
    enc= 524288.0
    x= pos % enc
    degree = float(float(x)/enc)*360.0  
    if pos < 0 :
        degree = degree - 360
    return round(degree, 2)


#--------------------------------------

output_velocity = (int)((1 * 524288) / 360) # 1 deg/s
basestep = 1000   # max 500  
PI=3.141592653589793
target=[0,0,0,0,0,0]


class ErobNode(Node): 
    def __init__(self):
        super().__init__("erob_node")
        self._logger.info("ARM Is Ready")

        # self.running =false      

        self.subscription1 = self.create_subscription(
            JointState,
            'arm/command',
            self.listener_callback,
            10)
        
        self.subscription2 = self.create_subscription(
            String,
            'arm/stop',
            self.listener_stop_callback,
            1)
        

        

        timer_period = 0.5
        self.publisher_ = self.create_publisher(JointState, 'arm/state', 10)    
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.publisher_2 = self.create_publisher(String, 'arm/run', 1)
        # self.timer2 = self.create_timer(timer_period, self.timer_callback_run)


    def get_degree(self):
        positions=[0,0,0,0,0,0]
        for i in range(number_of_slaves):
            input=upload(master.slaves[i],i,0x6064,0,'i',message='The current position')
            positions[i]=postodegree(input) 
        # print(upload(master.slaves[1],1,0x6064,0,'i',message='The current position'))
        return positions
    
    
    
    # def timer_callback_run(self):
    #     reached =0
    #     msg = String()
    #     msg.data = '0'

    #     for i in range(number_of_slaves):
    #         result = upload(master.slaves[i],i,0x6041,0,'H',message='Check state2') 
    #         if (result>> 10) % 2 == 0:
    #             msg.data = '1'
    #             break
        
    #     self.publisher_2.publish(msg)
   


    def timer_callback(self):

        base_Velocity = 0
        joints = JointState()
        
        joints.name = ['j1', 'j2', 'j3','j4', 'j5', 'j6']
        # joints.position = [0.763331, 0.415979, -1.728629, 1.482985, -1.135621, -1.674347, -0.496337]
        joints.position = self.get_degree()
        joints.velocity = [base_Velocity,base_Velocity,base_Velocity,base_Velocity,base_Velocity,base_Velocity]
        # joints.effort = []

        joints.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(joints)
        

    
    def listener_stop_callback(self,msg):
        print(f'Listener callback: ***** STOP *****')
        for i in range(number_of_slaves):
            releasebrake(i)
        # self.running = True
        


    def listener_callback(self, joints):
        
        # while self.running() == true:
        #     sleep(0.1)

        print(f'Listener callback: {joints}')
        for i in range(number_of_slaves):
            # print(f'{i}: {joints.position[i]}')
            move(i,joints.position[i],joints.velocity[i]) 
            # move(i,joints.position[i],0) 
        # print('end')


    def running(self):
        # check bit10 of status word, it is 1 if motor reached to target position
        for i in range(number_of_slaves):
            result = upload(master.slaves[i],i,0x6041,0,'H',message='Check state2') 
            if (result>> 10) % 2 ==0:
                return true
        
        return false
        

            
        
        # print(f'        Steps: {step}')
        # print(f'        Traget Positions: {target}')
        # print(f'        Traget Angles: {angle}')





#----------------------------------


def main(args=None):
    rclpy.init(args=args)
    # adapters = pysoem.find_adapters()

 
    mainapp()
    print('main done')
    
    node = ErobNode() 
    rclpy.spin(node)
    rclpy.shutdown()


# def main(args=None):
#     rclpy.init(args=args)
#     try:

#         # adapters = pysoem.find_adapters()

#         # for i, adapter in enumerate(adapters):
#         #     print('Adapter {}'.format(i))
#         #     print('  {}'.format(adapter.name))
#         #     print('  {}'.format(adapter.desc))

#         if  mainapp() > 0:
#             mainNode = ErobNode()
#             runNode = RunNode()
        


#             executor = MultiThreadedExecutor(num_threads=3) #3
#             executor.add_node(mainNode)
#             executor.add_node(runNode)

            
#             for i in range(0,number_of_slaves):
#                 data=struct.unpack('iiH',master.slaves[i].input)
#                 target[i] = data[0]
#                 step[i]=0

#             try:
#                 executor.spin()
#             finally:
#                 executor.shutdown()
#                 mainNode.destroy_node()
#                 runNode.destroy_node()
#                 actuatorNode.destroy_node()
#     finally:
#         rclpy.shutdown()
 
if __name__ == "__main__":
    main()


    
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [-0.5236,-0.5236,-0.5236,-0.5236,-0.5236,-0.5236]}" --once
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [0.5236,0.5236,0.5236,0.5236,0.5236,0.5236]}" --once
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [0, 0, 0, 0, 0, 0]}" --once
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [0, 0, 0, 0, 0, 0] ,velocity: [500, 500, 500, 500, 500, 500]}" --once
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [1.047, 1.047, 1.047, 1.047, 1.047, 1.047]}" --once

# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [1.62, 0.86, 0.28, 3.60, 1.04, 1.55],velocity:[ 1, 2, 2, 3, 4 , 4],effort: [ 1, 2, 2, 3, 4 , 4]}" --once
