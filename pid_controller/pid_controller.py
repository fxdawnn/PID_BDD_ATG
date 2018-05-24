import numpy as np
from scipy import stats
import pid

def import_txt(filename):
    arr = []
    file = open(filename,"r")
    line = file.read()
    line.strip()
    array = line.split()
    for i in array:
        arr.append(float(i))
    return arr
def lstsq(A, b):
    return np.linalg.solve(A.T.dot(A), A.T.dot(b))

def primary_tuning():
    """
    rough granularity tuning based on the static data using 
    MSE and linear fitting the data to Kp
    """
    # model output test data, angle velocity pid controller input
    arr_train_angles = import_txt("pid/train_angles.txt")
    # angle velocity pid controller output data, GTA input
    arr_train_steerings = import_txt("pid/train_steerings.txt")
    # model output test data, velocity pid controller input
    arr_train_speeds = import_txt("pid/train_speeds.txt")
    # velocity pid controller output data, GTA input
    arr_train_thottles = import_txt("pid/train_thottles.txt")
    arr_train_brakes = import_txt("pid/train_brakes.txt")

    arr_train_angles_next = arr_train_angles[1:]
    arr_train_angles = arr_train_angles[:-1]
    arr_train_speeds_next = arr_train_speeds[1:]
    arr_train_speeds = arr_train_speeds[:-1]


    error_angles = np.array(arr_train_angles) - np.array(arr_train_angles_next)
    error_speeds = np.array(arr_train_speeds) - np.array(arr_train_speeds_next)
    arr_train_thottles = np.array(arr_train_thottles[:-1])
    arr_train_steerings = np.array(arr_train_steerings[:-1])

    # Using OLS with error as input X and thottles and steerings as 
    # output y to solve the problem 
    #slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)
    Kp_v_primary,_,_,_,std_err_s_primary = stats.linregress(error_speeds, arr_train_thottles)
    Kp_a_primary,_,_,_,std_err_a_primary = stats.linregress(error_angles, arr_train_steerings)

    print(Kp_v_primary,Kp_a_primary)




if __name__ == '__main__':
    # tunning parameters
    # propotional for velocity and angle volocity
    Kp_v = 0    
    Kp_a = 0

    # integral
    Ki_v = 0
    Ki_a = 0

    # derivate gain
    Kd_v = 0
    Kd_a = 0

    # Setup max and min throttle and steering preventing overflow
    max_throttle = 1
    max_sttering = 1
    min_throttle = 0
    min_sttering = -1
    max_break = 1
    min_break = 0
    # Unit time interval between calc() calls
    sampletime = 0.01

    # PID for the angle velocity
    pid_angle_velocity = pid.PID(sampletime, Kp_a, Ki_a, Kd_a, min_sttering, max_sttering)
    # PID for the velocity
    pid_velocity = pid.PID(sampletime, Kp_v, Ki_v, Kd_v, min_throttle, max_throttle)

    print(pid_velocity.calc(2,1))
    print(pid_angle_velocity.calc(2,1))
    primary_tuning()