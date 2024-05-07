from gc import garbage
import math
from pickle import TRUE
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

number_of_slaves =6

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
# def shutdownAll():
#     for i,device in enumerate(master.slaves):        
#         shutdown(device,i) 
    
#     master.state = pysoem.OP_STATE
#     master.write_state()
#     # master.close()
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
            device_VelocityConfig(device,idx)
            # small_device_VelocityConfig(device,idx)
            
            download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
            # download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
            download(device,idx,0x60FF,0,value=0,message='Target velocity set 0') 
            pos = upload(device,idx,0x6064,0,'I',message='The current position')
            download(device,idx,0x607A,0,value=pos,message='Change The target position')

            # master.send_processdata()
            # master.receive_processdata(2000)
            # changeTo_OP_Ethercat(device) 

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
            print(f'Device {idx} Is Enable')
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
# --------------------------------
# def changeTo_OP_Enable2(device,idx):
#     data= struct.unpack('iiH',master.slaves[idx].input)
#     result = data[2]
#     # if getnbit(result,0,4) == 7:
#     #     print(f'Device {idx} Is Enable')
#     #     return

#     # download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
    
#     # while result is not Switch on disabled: wait
#     check =True
#     while check:
        
#         errorcode = upload(device,idx,0x603f,0,'H',message='Error Code')
#         if  errorcode > 0:
#             device_VelocityConfig(device,idx)
#             # small_device_VelocityConfig(device,idx)
            
#             download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
#             # download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
#             download(device,idx,0x60FF,0,value=0,message='Target velocity set 0') 
#             pos = data[0]
#             download(device,idx,0x607A,0,value=pos,message='Change The target position')

#             # master.send_processdata()
#             # master.receive_processdata(2000)
#             # changeTo_OP_Ethercat(device) 

#         data= struct.unpack('iiH',master.slaves[idx].input)
#         result = data[2] 
#         if getnbit(result,0,4) == 0b1000: 
#             download(device,idx,0x6040,0,value=0x0080,message='Clear Fault')
#             # download(device,idx,0x60FF,0,value=0,message='Target velocity set 0') 

#             # sleep(0.05)
#             # master.send_processdata()
#             # master.receive_processdata(2000)        
#             # sleep(0.05)
#             # data= struct.unpack('iiH',master.slaves[idx].input)
#             # result = data[2]

#         # result = upload(device,idx,0x6041,0,'H',message='Check state2')  
#         # print(getnbit(result,0,4))
#         if getnbit(result,0,4) == 7:
#             # print(f'Device {idx} Is Enable')
#             return 
#         val = getnbit(result,0,7) 
#         # print(f'{val:06b}')    
#         if (val == 0b0010000) or (val == 0b0110000):            
#             continue
         
#         if (val == 0b1010000) or (val == 0b1000000):
#         #    download(device,idx,0x60FF,0,value=0,message='Target velocity set 0')  
#            download(device,idx,0x6040,0,value=0x0006,message='Set ready to switch on')
#         #    sleep(0.05)

     
#         if (val == 0b0110001) or (val == 0b0100001): 
#             # download(device,idx,0x60FF,0,value=0,message='Target velocity set 0')  
#             download(device,idx,0x6040,0,value=0x0007,message='Set switch on')
#             # sleep(0.05)        
        
#         if (val == 0b0110011) or (val == 0b0100011):
#             # download(device,idx,0x60FF,0,value=0,message='Target velocity set 0')  
#             download(device,idx,0x6040,0,value=0x000f,message='Set Enable operation')
#             # sleep(0.05)    
#         # master.send_processdata()
#         # master.receive_processdata(2000)  


#     # result = upload(device,idx,0x6041,0,'H')
#     # print(f'\t New State: {hex(device.state)} => {device.state:08b}')
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
    download(device,idx,0x3b60,0,value=60000,message='Set: Maximum ')

# ---------------------------------- 
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
                device_PDO_setup(device)

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
    return round(degree, 2)

#--------------------------------------

def caltargetposition(pos,targetdegree):
    """ Calcute target position from current position and target degree

    Parameters
    ----------
        pos: int 
            Current position
        targetdegree: float
            target degree
    """   
    enc= 524288.0
    currentdegree= postodegree(pos)
    diff = targetdegree - currentdegree

    if currentdegree < targetdegree:
        if diff> 180:
            diff = diff - 360

    if currentdegree > targetdegree:
        if diff  <-180:
            diff = diff + 360

    target= pos + (diff * (enc/360.0))
    return int(target)

#--------------------------------------
def change_position(angle):
    """ Move Arm to input angles

    Parameters
    ----------
        angle: list of angles 
    """   
    # master.send_processdata()
    # master.receive_processdata(2000)
    # print(f' \n Output Is:  {master.slaves[0].output.hex()}')
    # print(f'Target: {angle}')

    data=[]
    target=[]
    currentangle=[]
    flag=[]

    for i in range(number_of_slaves):
        data.append(struct.unpack('iiH',master.slaves[i].input))

    # print('Master state: ',hex(master.read_state()))    
    # for i in range(number_of_slaves):    
    #     print(f'slave {i} state: ',hex(master.slaves[i].state) )
    #     # print(f'slave {i} state: ',hex(master.slaves[i].state) ,'----', hex(master.slaves[i].state_check(master.slaves[i].state)))

    # master.send_processdata()
    # master.receive_processdata(1000)
    # sleep(0.01)
    # # # set target position equal actual position
    for i in range(number_of_slaves):
        master.slaves[i].output = struct.pack('iii', data[i][0],0,brake_state) 
    
    # master.send_processdata()
    # master.receive_processdata(10)
    # sleep(0.01)


    print(data)
    for i in range(number_of_slaves):
        currentangle.append(postodegree(data[i][0]))
        target.append (caltargetposition(data[i][0],angle[i]))
       
    print(f'Current: {currentangle}')
    # print(target)

    output_velocity = (int)((1 * 524288) / 360) # 5 deg/s
    # output_velocity = 10
    basestep = 20   #(120 very good)   (200 vibration: a bit )
    step=[]
    for i in range(number_of_slaves):
        if abs(target[i] - data[i][0]) == 0:
            flag.append(1)
        else:
            flag.append(0)

        if target[i] < data[i][0]:
            step.append(basestep * -1)
        else:
            step.append(basestep)
    print(step)

    print(f'Flags: {flag}')
    # change machine state
    for g in range(number_of_slaves):
        if flag[g] == 0:
            if getnbit(data[g][2],0,4) != 7:
                print(f'Change to Enable {g}')
                changeTo_OP_Enable(master.slaves[g],g)



    # master.send_processdata()
    # master.receive_processdata(10)
    # sleep(0.01)
    
    # for i in range(number_of_slaves):
    #     print(f'Slave {i}=> Input:  {master.slaves[i].input.hex()} -  Output:  {master.slaves[i].output.hex()}')
    
  
    if data[0] == 0:
        print('Zero')
        return

    # for i in range(number_of_slaves):    
    #     if hex(master.slaves[i].state_check(master.slaves[i].state)) != pysoem.OP_STATE:
    #         return
    for i in range(number_of_slaves):
        if flag[i] == 0:
            while master.slaves[i].state != pysoem.OP_STATE:
                shutdown(master.slaves[i])
                changeTo_OP_Ethercat(master.slaves[i],i)


   

    print('start')
    i=0
    run=True
    while run:
        i+=1
        master.receive_processdata(10)
                
        # sleep(0.01)
        


        # for m in range(number_of_slaves):
        
        for n in range(number_of_slaves):
            data[n]= struct.unpack('iiH',master.slaves[n].input)
            if (flag[n] == 0) and (i % 999 == 0):
                    if master.slaves[n].state != 0x8 or master.slaves[n].state_check(master.slaves[n].state) != 0x8:
                        # print(f'slave {n} state: ',hex() ,'----', hex(master.slaves[n].state_check(master.slaves[n].state)))
                        flag[n]=-1
                        # print(f'flags: {flag} ----- {printAngles()} ---- {data}')
            if abs(data[n][0] - target[n]) > (2*basestep) and flag[n] == 0: 
                # if getnbit(data[n][2],10,1) == 1:                                  
                    master.slaves[n].output = struct.pack('iii', data[n][0] + step[n],output_velocity,brake_state)

            else:
               
                if flag[n] != 1:
                    flag[n]=1
                    # printAngles()  


            for g in range(number_of_slaves):
                if flag[g] != 1:
                    if getnbit(data[g][2],0,4) != 7:
                        print(f'1  device :{g}')
                        changeTo_OP_Enable(master.slaves[g],g)  
                    # if flag[g]== -1:
                    #     print(f'2  device :{n}') 
                    #     # if master.slaves[g].state != pysoem.OP_STATE:
                    #     shutdown(master.slaves[g],g)
                    #     changeTo_OP_Ethercat(master.slaves[g],garbage)
                             

        master.send_processdata()
        reach =0
        for d in range(number_of_slaves):
            if flag[d] != 0:
                reach+=1
        if reach == number_of_slaves:
            # print(flag)
            run = False
            # print(flag)
            # printAngles() 
           

    # print('Master state: ',hex(master.read_state()))
    # for i in range(number_of_slaves):    
    #     print(f'slave {i} state: ',hex(master.slaves[i].state) )
    #     # print(f'slave {i} state: ',hex(master.slaves[i].state) ,'----', hex(master.slaves[i].state_check(master.slaves[i].state)))

    # for i in range(number_of_slaves):
    #     print(f'Slave {i}=> Input:  {master.slaves[i].input.hex()} -  Output:  {master.slaves[i].output.hex()}')
    

    print('=================================================')

#--------------------------------------
def printAngles():
    """ Show current angles
    """   
    p=[]
    for w in range(number_of_slaves):            
        input=struct.unpack('iiH',master.slaves[w].input)
        p.append(postodegree(input[0])) 
    print(p) 
    return p

#--------------------------------------

output_velocity = (int)((1 * 524288) / 360) # 1 deg/s
basestep = 1000   # max 500  
PI=3.141592653589793
target=[0,0,0,0,0,0]
step=[0,0,0,0,0,0]
maxstep=[0,0,0,0,0,0]
incstep=[0,0,0,0,0,0]
data=[0,0,0,0,0,0]
olddata=[0,0,0,0,0,0]


class ErobNode(Node): 
    def __init__(self):
        super().__init__("erob_node")
        self._logger.info("ARM Is Ready")

        self.running =false      

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
        
        self.subscription3 = self.create_subscription(
            String,
            'arm/disengage',
            self.listener_disengage_callback,
            1)
        

        timer_period = 0.01
        self.publisher_ = self.create_publisher(JointState, 'arm/state', 10)    
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publisher_2 = self.create_publisher(String, 'arm/run', 1)
        self.timer2 = self.create_timer(timer_period, self.timer_callback_run)


    def get_radians(self):
        positions=[0,0,0,0,0,0]
        for i in range(number_of_slaves):
            input=struct.unpack('iiH',master.slaves[i].input) 
            positions[i]=( postodegree(input[0]) * PI ) / 180
        return positions
    

    
    def timer_callback_run(self):
        reached =0
        msg = String()
        msg.data = '1'

        for i in range(number_of_slaves):
            if step[i]==0 :
                reached+=1
        if reached == number_of_slaves:
            self.running == false
            msg.data ='0'

        self.publisher_2.publish(msg)
   


    def timer_callback(self):

        base_Velocity = 10
        joints = JointState()
        
        joints.name = ['j1', 'j2', 'j3','j4', 'j5', 'j6']
        # joints.position = [0.763331, 0.415979, -1.728629, 1.482985, -1.135621, -1.674347, -0.496337]
        joints.position = self.get_radians()
        joints.velocity = [base_Velocity,base_Velocity,base_Velocity,base_Velocity,base_Velocity,base_Velocity]
        joints.effort = []

        joints.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(joints)

    
    def listener_stop_callback(self,msg):
        print(f'Listener callback: ***** STOP *****')
        for i in range(number_of_slaves):
            data=struct.unpack('iiH',master.slaves[i].input)
            target[i] = data[0]
            brake_state = brake_Engage
            step[i]=0
        # self.running = True
        

    def listener_disengage_callback(self,msg):
        print(f'Listener callback: ***** Disengage  *****')
        for i in range(number_of_slaves):
            brake_state = brake_Disengage
        # self.running = False
        


    def listener_callback(self, joints):
        
        self.check_run()

        # angle=[0,0,0,0,0,0]
        sleep(0.01)
        print(f'Listener callback: {joints}')
        # if self.check_collision(joints.position):
        #     return
        # basestep = int(joints.velocity[0])
        for i in range(number_of_slaves):
            data=struct.unpack('iiH',master.slaves[i].input)
            # angle[i]= (((joints.position[i] * 180 )/ PI ) + 360)%360            
            # target[i] = caltargetposition(data[0],angle[i]) 
            target[i] = caltargetposition(data[0],joints.position[i]) 

            if target[i] < data[0]:
                step[i]=-200
                maxstep[i]=(int(joints.velocity[i]) * -1)
                incstep[i]=-20
            else:
                step[i]= 200
                maxstep[i]=int(joints.velocity[i]) 
                incstep[i]=20

            if abs(  data[0] - target[i]) < (int(joints.velocity[i]) ):
                step[i] = 0
        print(target)
        print(step)
    
    def check_run(self):
        while self.running == true:
            reached =0
            for i in range(number_of_slaves):
                if abs(step[i]) <= 50 :
                    reached+=1
            if reached == number_of_slaves:
                self.running == false
                print('Arm is ready')
                return
            sleep(0.2)
            
        
        # print(f'        Steps: {step}')
        # print(f'        Traget Positions: {target}')
        # print(f'        Traget Angles: {angle}')

    # def check_collision(self,joints):
    #     joint_lengh=45
    #     minimum_y=12
    #     newj2= joints[1]
    #     angle_j2 = 90 - newj2
    #     # print(angle_j2)

    #     x1 = math.sqrt(joint_lengh**2 - (minimum_y - joint_lengh * math.sin(math.radians(angle_j2)))**2)
    #     x= x1 + joint_lengh * math.cos(math.radians(angle_j2))

    #     t = math.degrees(math.asin(minimum_y/x))
    #     inside_angle = 90 - (newj2+t)

    #     angle_j3 = 180 - (inside_angle * 2)

    #     newj3 = joints[2]
    #     # print(f'J2: {newj2} ')

    #     # print('==============')
    #     if (angle_j3-180) > newj3 and newj3 < angle_j3:
    #         # print(f"Collision -------------J2: {newj2} ---------- J3: {newj3} ")
    #         return true

    #     return false



#--------------------------------------   
class RunNode(Node):
    def __init__(self):
        super().__init__("run_node")
        self._logger.info("Run Node is init")
        self.count=0 
        self.freerun=20

        timer_period = 0.01
        self.refreshtime=timer_period * 100
        self.basestep=400

        self.timer2 = self.create_timer(timer_period, self.free_run)
        # print(step)
        # print(data)
        # print(target)

    def free_run(self):
        """ infinity loop for change position

        use list of step and list of target (Global values) to move the Arm
        """       
        self.count+=1
        master.receive_processdata(self.refreshtime)
        # sleep(0.01)
        for n in range(number_of_slaves):
            data[n]= struct.unpack('iiH',master.slaves[n].input)
           
            if abs(data[n][0] - target[n]) < (self.basestep):
                step[n]=0

            # if (abs(data[n][0] - target[n])/1450) < (self.basestep):
                
            # if step[n]==0:
            #     newpos = target[n] + self.freerun[n]
            #     self.freerun[n] *= -1
            #     # print(step)
            #     # print(data)
            #     # print(target)
           

            
            if step[n]==0:
                master.slaves[n].output = struct.pack('iii', target[n],0,brake_state)
            else:
                
                # if n==1 :
                #     if self.check_collision(data[1][0]+ step[1],data[2][0]+ step[2]) == true:
                #         master.slaves[n].output = struct.pack('iii', data[n][0] ,output_velocity,brake_state)
                #         continue
                master.slaves[n].output = struct.pack('iii', data[n][0] + step[n] ,output_velocity,brake_state)

            #     self.freerun*=-1
            # else:
            #     master.slaves[n].output = struct.pack('ii', data[n][0] ,0)
            #     print(data)
        
        # print(data)
                    
        master.send_processdata()

        for k in range(number_of_slaves):
            if step[k] != 0:
                # data[k]= struct.unpack('iiH',master.slaves[k].input)
                
                if  abs(data[k][0] - target[k]) < 1400  and step[k]>200:
                    step[k] = step[k] - (incstep[k]*5)
                    maxstep[k] = 0
                else:
                    if abs(step[k]) < abs(maxstep[k]):
                        step[k] = step[k] + incstep[k]
            
        # print(step)
        
        # if self.count % 2 == 1: 
        #     for g in range(number_of_slaves):             
        #         if abs(step[g]) < self.freerun:
        #             if target[g] < data[g][0]:
        #                 step[g]=(self.freerun * -1)
        #             else:
        #                 step[g]=self.freerun

        if self.count % 100 == 1:    
            for g in range(number_of_slaves):          
                if master.slaves[g].state != 0x8 or master.slaves[g].state_check(master.slaves[g].state) != 0x8:
                    # print(f'slave {g} state: ----',master.slaves[g].state, hex(master.slaves[g].state_check(master.slaves[g].state)))
                    # flag[n]=-1
                    shutdown(master.slaves[g])
                    changeTo_OP_Ethercat(master.slaves[g],g) 
                    changeTo_OP_Enable(master.slaves[g],g) 

                if getnbit(data[n][2],0,4) != 7:
                    # print(f'====  device :{g} {data[g]}')
                    changeTo_OP_Enable(master.slaves[g],g)  

   
    # def check_collision(self,posj2,posj3):
    #     joint_lengh=45
    #     minimum_y=12
    #     newj2= postodegree(posj2)
    #     angle_j2 = 90 - newj2
    #     # print(angle_j2)

    #     x1 = math.sqrt(joint_lengh**2 - (minimum_y - joint_lengh * math.sin(math.radians(angle_j2)))**2)
    #     x= x1 + joint_lengh * math.cos(math.radians(angle_j2))

    #     t = math.degrees(math.asin(minimum_y/x))
    #     inside_angle = 90 - (newj2+t)

    #     angle_j3 = 180 - (inside_angle * 2)

    #     newj3 = (postodegree(posj3)%360)-360
    #     # print(f'J2: {newj2} ')
    #     # print(f'J3: {postodegree(posj3)} ')
    #     # print('==============')
    #     if (angle_j3-180) > newj3 and newj3 < angle_j3:
    #         # print(f"Collision -------------J2: {newj2} ---------- J3: {newj3} ")
    #         return true

    #     return false



#-----------------------------------------

class ActuatorNode(Node):
    def __init__(self):
        super().__init__('serial')
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=0.5)   # Hand
        # self.serial_port2 = serial.Serial('/dev/ttyUSB1', baudrate=9600, timeout=0.5)  # LED & Buzzer1
        self.get_logger().info('UART Sensor Node has been started')
        self.actuator_pub = self.create_publisher(String, 'actuator/state', 1)
        self.timer = self.create_timer(0.5, self.receive_sensor_data)
        self.count=0
        self.port1=False
        self.port2=False


        self.subscription = self.create_subscription(
                String,
                'hand/command',
                self.serial_listener_callback,
                1)
        
        # self.subscription_LED = self.create_subscription(
        #         String,
        #         'led/command',
        #         self.led_serial_listener_callback,
        #         1)

        # self.subscription_Buzzer = self.create_subscription(
        #         String,
        #         'buzzer/command',
        #         self.buzzer_serial_listener_callback,
        #         1)

    def serial_listener_callback(self,msg):
        # if self.port1 == False:
        if msg.data=="pickup":
            print("Pickup Command")
            self.serial_port.write(bytes("A", 'utf-8'))
            self.port1 = True
        if msg.data=="dropoff":  
            print("Dropoff Command")
            self.serial_port.write(bytes("B", 'utf-8'))
            self.port1 = True
        
            

       
    # def led_serial_listener_callback(self,msg):
    #     if self.port2 == False:
    #         print("LED Command")
    #         print(msg.data)
    #         value ="L"+ msg.data          # A    
    #         # self.serial_port2.write(bytes(value, 'utf-8'))
    #         self.serial_port.write(bytes(value, 'utf-8'))
    #         self.port2 = True
    
    # def buzzer_serial_listener_callback(self,msg):
    #     if self.port2 == False:
    #         print("Buzzer Command")
    #         print(msg.data)  
    #         value ="R"+ msg.data          #B    
    #         # self.serial_port2.write(bytes(value, 'utf-8'))
    #         self.serial_port.write(bytes(value, 'utf-8'))
    #         self.port2 = True



    # 2 Arduino
    def receive_sensor_data(self):   
        msg = String()
       

        # while self.serial_port.in_waiting<3 or self.serial_port2.in_waiting <3:
        #     time.sleep(0.01)

        if self.serial_port.in_waiting >= 2:
            # print('waiting for serial port...')

            actuatordata = self.serial_port.read(4).decode()
            print(actuatordata)
            # if actuatordata == "ok":
            #     msg.data = 'done1' 
            self.port1 = False
            #     print("done1")
            msg.data = actuatordata

            self.actuator_pub.publish(msg)
        
        # else:
        #     if self.serial_port2.in_waiting >= 2:
        #     # print('waiting for serial port...')
           
        #         actuatordata2 = self.serial_port2.read(4).decode()
        #         print(actuatordata2) 
        #         # if actuatordata2 == "ok":
        #         #     msg.data = 'done2' 
        #         self.port2 = False
        #         #     print("done2")
        #         msg.data = actuatordata2
                
        #         self.actuator_pub.publish(msg)
                
        #     # else:
        #     #     msg.data = 'working' 
        #     #     self.actuator_pub.publish(msg)

# # 1 Arduino
#     def receive_sensor_data(self):   
#         msg = String()
       
#         if self.serial_port.in_waiting >= 2:
#             # print('waiting for serial port...')

#             actuatordata = self.serial_port.read(4).decode()
#             print(actuatordata)
#             # if actuatordata == "ok":
#             #     msg.data = 'done1' 
#             self.port1 = False
#             #     print("done1")
#             msg.data = actuatordata

#             self.actuator_pub.publish(msg)
        
        
#             if actuatordata == "ok":
#                 msg.data = 'done2' 
#                 self.port2 = False
#             #     print("done2")
            
                
            

      

#----------------------------------


# def main(args=None):
#     rclpy.init(args=args)
#     # adapters = pysoem.find_adapters()

 
#     mainapp()
#     print('main done')
    
#     node = ErobNode() 
#     rclpy.spin(node)
#     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:

        # adapters = pysoem.find_adapters()

        # for i, adapter in enumerate(adapters):
        #     print('Adapter {}'.format(i))
        #     print('  {}'.format(adapter.name))
        #     print('  {}'.format(adapter.desc))

        if  mainapp() > 0:
            mainNode = ErobNode()
            runNode = RunNode()
            actuatorNode = ActuatorNode()


            executor = MultiThreadedExecutor(num_threads=3) #3
            executor.add_node(mainNode)
            executor.add_node(runNode)
            executor.add_node(actuatorNode)
            
            for i in range(0,number_of_slaves):
                data=struct.unpack('iiH',master.slaves[i].input)
                target[i] = data[0]
                step[i]=0

            try:
                executor.spin()
            finally:
                executor.shutdown()
                mainNode.destroy_node()
                runNode.destroy_node()
                actuatorNode.destroy_node()
    finally:
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()


    
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [-0.5236,-0.5236,-0.5236,-0.5236,-0.5236,-0.5236]}" --once
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [0.5236,0.5236,0.5236,0.5236,0.5236,0.5236]}" --once
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [0, 0, 0, 0, 0, 0]}" --once
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [0, 0, 0, 0, 0, 0] ,velocity: [500, 500, 500, 500, 500, 500]}" --once
# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [1.047, 1.047, 1.047, 1.047, 1.047, 1.047]}" --once

# ros2 topic pub --once /arm/command sensor_msgs/msg/JointState "{name: ["j1", "j2", "j3", "j4", "j5", "j6"],position: [1.62, 0.86, 0.28, 3.60, 1.04, 1.55],velocity:[ 1, 2, 2, 3, 4 , 4],effort: [ 1, 2, 2, 3, 4 , 4]}" --once