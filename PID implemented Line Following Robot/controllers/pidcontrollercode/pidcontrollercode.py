
from controller import Robot


robot = Robot()


timestep = int(robot.getBasicTimeStep())

ds=[]
wheel =[]
wheel_name =["front_right_wheel","front_left_wheel","back_right_wheel","back_left_wheel"]
ds_name=["front_right","front_left","extreme_right","extreme_left","front_sensor","right_sensor","left_sensor"]
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


while robot.step(timestep) != -1:
    
    for i in range(len(ds)):
        ds_val[i] = ds[i].getValue()
        print(f"{ds_name[i]}:{ds_val[i]}\n")
        
    if 1000 in ds_val[:2]:
        if ds_val[0]>950 and ds_val[1]>950 and ds_val[2]<950 and ds_val[3]<950:
            setSpeed(5,0)
            print("case 0")
        elif ds_val[0]>950 and ds_val[1]>950 and ds_val[2]>950 and ds_val[3]<950:
            setSpeed(2,-2)
            print("case 1")
        elif ds_val[0]>950 and ds_val[1]>950 and ds_val[2]<950 and ds_val[3]>950:
            setSpeed(2,2)
            print("case 3")
        elif ds_val[0]<950 and ds_val[1]>950:
            setSpeed(5,2)
            print("case 4")
        elif ds_val[0]>950 and ds_val[1]<950:
            setSpeed(5,-2)
            print("case 5")
        elif ds_val[0]>950 and ds_val[1]>950 and ds_val[2]>950 and ds_val[3]>950:
            print("case 6")
        else:
            speed(0,0)
    else:
        if ds_val[4]!= 1000:
            setSpeed(0,3)
            print("case 7")
        else:
            error = ds_val[6] - 700
            rectify = pid(error)
            setSpeed(5,-rectify)
            print("case 8")
                          
  
    pass

