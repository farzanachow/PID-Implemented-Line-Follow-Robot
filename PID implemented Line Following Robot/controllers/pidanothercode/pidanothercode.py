"""pid_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

ds=[]
wheel =[]
wheel_name =["front_right_motor","front_left_motor","back_right_motor","back_left_motor"]
ds_name=["ir_right","ir_left","ir_ext_right","ir_ext_left","front_sensor","right_sensor","left_sensor"]
ds_val = [0]*len(ds_name)
for i in ds_name:
    ds.append(robot.getDevice(i))
    ds[-1].enable(timestep)
for i in wheel_name:
    wheel.append(robot.getDevice(i))
    wheel[-1].setPosition(float("inf"))
    wheel[-1].setVelocity(0.0)

last_error = intg = prop = diff = 0
Kp =0.005
Ki = 0 
Kd = 0.15

def pid(error):
    global last_error, intg, prop ,diff, Kp,Kd,Ki
    prop = error
    intg = error  + intg 
    diff = error  - last_error
    balance = (Kp*prop) + (Kd*diff) + (Ki*intg)
    last_error = error
    return balance
    
def setSpeed(base_speed,balance):
    
    wheel[0].setVelocity(base_speed+balance)
    wheel[1].setVelocity(base_speed-balance)
    wheel[2].setVelocity(base_speed+balance)
    wheel[3].setVelocity(base_speed-balance)
    
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    for i in range(len(ds)):
        ds_val[i] = ds[i].getValue()
        print(f"{ds_name[i]}:{ds_val[i]}\n")
        
    
    if ds_val[4]!= 1000:
        setSpeed(0,3)
        print("case 7")
    else:
        error = ds_val[6] - 700
        rectify = pid(error)
        setSpeed(5,-rectify)
        print("case 8")
                          
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.