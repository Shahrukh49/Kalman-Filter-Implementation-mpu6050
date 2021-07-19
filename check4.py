from mpu6050 import mpu6050
import math

mpu = mpu6050(0x68)

#######################################
# Programmed by Engr. Shahrukh Saleem
#
# There are 3 Equations for applying Kalman Filter on single parameter
#
# #Equation 1: Calculating Kalman Gain 
# Kalman Gain = Error in Estimated Value / (Error in Estimated Value + Error in Measured Sensor Value)
#
# #Equation 2: Calculation the Estimated Value also called Filtered Value
# Estimated Value = Previous Estimated Value + Kalman Gain x (Measured Value of Sensor - Previous Estimated Value)
#
# #Equation 3: Calculating Error in Estimated Value
# Error in Estimated Value = [1 - Kalman Gain] x Error in Previous Estimated Value
#
#######################################

E_MEA = 4                                   # ASSUMING ERROR OF 4 DIVISION FOR MEASUREMENT
Initial_EST = 68                            # INITIAL ASSUMPTION FOR ESTIMATED VALUE, YOU CAN PUT ANY VALUE
Initial_E_Est = 2                           # INITIAL ASSUMPTION FOR ERROR IN ESTIMATED VALUE, YOU CAN PUT ANY VALUE
radToDeg = 57.2957786

#THE 3 EQUATIONS ARE WRITTEN IN A FUNCTION
def KalmanFilter(MEA, EST_prev, E_Est_prev):
    KG = E_Est_prev / (E_Est_prev + E_MEA)
    EST = EST_prev + KG*(MEA - EST_prev)
    E_Est = (1-KG)*(E_Est_prev)

    #IF THE DEVIATION BETWEEN MEASURED AND ACTUAL VALUE IS LARGE
    deviation = abs(EST - MEA) / (EST + MEA) * 100

    #IF THE ERROR IS GREATER THAN 25%, MAKE ESTIMATED ERROR LARGE
    if (deviation > 25):
        E_Est = 10
    return KG, EST, EST_prev, E_Est, E_Est_prev


def getMPUdata():
    gyro = mpu.get_gyro_data()
    accel = mpu.get_accel_data()

    roll = math.atan2(accel['y'],accel['z']) * radToDeg
    pitch = math.atan(-accel['x']/math.sqrt((accel['y']**2)+(accel['z']**2))) * radToDeg

    return roll, pitch, accel

        

#GETTING DATA FROM GYRO
r, p, acceler = getMPUdata()

#GETTING VALUE FOR INITIAL ASSUMPTIONS, 1ST ESTIMATED VALUE AND ERROR VALUE WILL BE GENERATED
KalmanGainX, ValueX, prevValueX, ErrorX, PrevErrorX = KalmanFilter(r, Initial_EST, Initial_E_Est)
KalmanGainY, ValueY, prevValueY, ErrorY, PrevErrorY = KalmanFilter(p, Initial_EST, Initial_E_Est)

KalmanGainXa, ValueXa, prevValueXa, ErrorXa, PrevErrorXa = KalmanFilter(acceler['x'], Initial_EST, Initial_E_Est)
KalmanGainYa, ValueYa, prevValueYa, ErrorYa, PrevErrorYa = KalmanFilter(acceler['y'], Initial_EST, Initial_E_Est)
KalmanGainZa, ValueZa, prevValueZa, ErrorZa, PrevErrorZa = KalmanFilter(acceler['z'], Initial_EST, Initial_E_Est)

while(1): 
    #GETTING DATA FROM GYRO & ACCELEROMETER
    r, p, acceler = getMPUdata()
    
    #AFTER 1ST ESTIMATED VALUE, THE FILTER WILL AGAIN USE THAT VALUE TO GET ANOTHER TUNED VALUE
    #THE PROCESS IS CONTINUED UNTIL THE KALMAN FILTER GETS ESTIMATED VALUE CLOSED TO ACTUAL VALUE
    KalmanGainX, ValueX, prevValueX, ErrorX, PrevErrorX = KalmanFilter(r, ValueX, ErrorX)
    KalmanGainY, ValueY, prevValueY, ErrorY, PrevErrorY = KalmanFilter(p, ValueY, ErrorY)

    print("-------------------------- GYRO DATA --------------------------------")
    print("ROLL =",r,"Filtered ROLL =",ValueX,"Gain",KalmanGainX,"Error",ErrorX)
    print("PITCH=",p,"Filtered PITCH=",ValueY,"Gain",KalmanGainY,"Error",ErrorY)

    KalmanGainXa, ValueXa, prevValueXa, ErrorXa, PrevErrorXa = KalmanFilter(acceler['x'], ValueXa, ErrorXa)
    KalmanGainYa, ValueYa, prevValueYa, ErrorYa, PrevErrorYa = KalmanFilter(acceler['y'], ValueYa, ErrorYa)
    KalmanGainZa, ValueZa, prevValueZa, ErrorZa, PrevErrorZa = KalmanFilter(acceler['z'], ValueZa, ErrorZa)

    print("---------------------- ACCEROMETER DATA -----------------------------")
    print("X=",accel['x'],"Filtered X=",ValueXa,"Gain",KalmanGainXa,"Error",ErrorXa)
    print("Y=",accel['y'],"Filtered Y=",ValueYa,"Gain",KalmanGainYa,"Error",ErrorYa)
    print("Z=",accel['z'],"Filtered Z=",ValueZa,"Gain",KalmanGainZa,"Error",ErrorZa)
