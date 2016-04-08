""" 
Kalman_test.py 

This file is used to test Kalman filter
"""

from __future__ import division
from Kalman import KalmanStatic1D, Custom3DKalman, simple_filter
import csv


def testCustom3DKalman():
    with open('input.csv', 'rb') as inputFile:
        with open('output.csv', 'w') as outputFile:
            outputFieldnames = [ 'time',
                                'lasers_x', 'lasers_y', 'lasers_z', 'lasers_yaw', \
                                'local_vel_x', 'local_vel_y', 'local_vel_z', 'local_vel_yaw', \
                                'accel_no_gravity_x', 'accel_no_gravity_y', 'accel_no_gravity_z', \
                                'filtered_x', 'filtered_y', 'filtered_z', 'filtered_yaw' ]

            fieldnames = ['time', 'setpoint_x', 'setpoint_y', 'setpoint_z', 'setpoint_yaw', \
                      'local_x', 'local_y', 'local_z', 'local_roll', 'local_pitch', 'local_yaw', \
                      'lasers_x', 'lasers_y', 'lasers_z', 'lasers_yaw', \
                      'raw_x_1', 'raw_x_2', 'raw_y_1', 'raw_y_2','raw_z_1', 'raw_z_2', \
                      'local_vel_x', 'local_vel_y', 'local_vel_z', 'local_vel_yaw', \
                      'accel_lin_x', 'accel_lin_y', 'accel_lin_z', \
                      'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', \
                      'filtered_x', 'filtered_y', 'filtered_z', 'filtered_yaw', \
                      'accel_no_gravity_x', 'accel_no_gravity_y', 'accel_no_gravity_z']

            data_reader = csv.DictReader(inputFile, fieldnames=fieldnames, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            data_writer = csv.writer(outputFile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

            Kalman = Custom3DKalman(0.05, 0.0872665, [0,0,0,0])
            lasttime = 0.0
            count = 0
            for row in data_reader : 
                if(count == 2):
                    Kalman = Custom3DKalman(0.025, 0.0872665, [float(row['lasers_x']),float(row['lasers_y']),float(row['lasers_z']),float(row['lasers_yaw'])], 0.01)
                    lasttime = float(row['time'])
                    data_writer.writerow(outputFieldnames)
                if(count > 2):
                    # target = (X1, X2, Y1, Y2, Z1, Z2, Yaw1, Yaw2) in m/s and rad
                    # linearSpeed = (Vx, Vy, Vz) in m/s
                    # LinearAcceleration = (Ax, Ay, Az) in m/s/s
                    # AngularSpeed = yawRate in rad/s
                    # dt in seconds
                    # Kalman.next(target, linearSpeed, linearAcceleration, angularSpeed, dt)
                    Measures =  [float(row['lasers_x']), float(row['lasers_x']), \
                                float(row['lasers_y']), float(row['lasers_y']), \
                                float(row['lasers_z']), float(row['lasers_z']), \
                                float(row['lasers_yaw']), float(row['lasers_yaw'])]

                    linearSpeed = [ float(row['local_vel_x']), \
                                    float(row['local_vel_y']), \
                                    float(row['local_vel_z'])]

                    angularSpeed = [float(row['local_vel_yaw'])]

                    linearAcceleration = [float(row['accel_no_gravity_x']), \
                                          float(row['accel_no_gravity_y']), \
                                          float(row['accel_no_gravity_z'])]

                    dt = float(row['time']) - lasttime

                    Xkp, k = Kalman.next(Measures, linearSpeed, linearAcceleration, angularSpeed[0], dt)

                    print Xkp
                    

                    lasttime = float(row['time'])

                    data_writer.writerow([row['time'], row['lasers_x'], row['lasers_y'], row['lasers_z'], row['lasers_yaw'], \
                                row['local_vel_x'], row['local_vel_y'], row['local_vel_z'], row['local_vel_yaw'], \
                                row['accel_no_gravity_x'], row['accel_no_gravity_y'], row['accel_no_gravity_z'], \
                                str(Xkp[0]), str(Xkp[1]), str(Xkp[2]), str(Xkp[3])])

                count = count +1


def test1DKalman():
    with open('input.csv', 'rb') as inputFile:
        with open('output.csv', 'w') as outputFile:
            outputFieldnames = ['raw_x_1', 'raw_x_2', 'raw_y_1', 'raw_y_2','raw_z_1', 'raw_z_2']
            fieldnames = ['time', 'setpoint_x', 'setpoint_y', 'setpoint_z', 'setpoint_yaw', 'local_x', 'local_y', 'local_z', 'local_roll', 'local_pitch', 'local_yaw', 'lasers_x', 'lasers_y', 'lasers_z', 'lasers_yaw', 'raw_x_1', 'raw_x_2', 'raw_y_1', 'raw_y_2','raw_z_1', 'raw_z_2']
        
            data_reader = csv.DictReader(inputFile, fieldnames=fieldnames, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            data_writer = csv.writer(outputFile, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)

            Kalman1 = KalmanStatic1D(4, 2, 75, 68)
            Kalman2 = KalmanStatic1D(4, 2, 75, 68)
            Kalman3 = KalmanStatic1D(4, 2, 75, 68)
            Kalman4 = KalmanStatic1D(4, 2, 75, 68)
            Kalman5 = KalmanStatic1D(4, 2, 75, 68)
            Kalman6 = KalmanStatic1D(4, 2, 75, 68)

            output = [0,0,0,0,0,0]
            count = 0

            for row in data_reader : 
                if(count == 2):
                    Kalman1 = KalmanStatic1D(2.5, 5, float(row['raw_x_1']), float(row['raw_x_1']))
                    Kalman2 = KalmanStatic1D(2.5, 5, float(row['raw_x_2']), float(row['raw_x_2']))
                    Kalman3 = KalmanStatic1D(2.5, 5, float(row['raw_y_1']), float(row['raw_y_1']))
                    Kalman4 = KalmanStatic1D(2.5, 5, float(row['raw_y_2']), float(row['raw_y_2']))
                    Kalman5 = KalmanStatic1D(2.5, 5, float(row['raw_z_1']), float(row['raw_z_1']))
                    Kalman6 = KalmanStatic1D(2.5, 5, float(row['raw_z_2']), float(row['raw_z_2']))
                    data_writer.writerow(outputFieldnames)
                if(count > 711):
                    output[0] = Kalman1.next(float(row['raw_x_1']))
                    output[1] = Kalman2.next(float(row['raw_x_2']))
                    output[2] = Kalman3.next(float(row['raw_y_1']))
                    output[3] = Kalman4.next(float(row['raw_y_2']))
                    output[4] = Kalman5.next(float(row['raw_z_1']))
                    output[5] = Kalman6.next(float(row['raw_z_2']))
                    data_writer.writerow([ row['raw_x_1'], output[0], row['raw_x_2'], output[1], row['raw_y_1'], output[2], row['raw_y_2'], output[3], row['raw_z_1'], output[4], row['raw_z_2'], output[5] ] )
                count = count +1

def testSimpleFilter():
    with open('input.csv', 'rb') as inputFile:
        with open('output.csv', 'w') as outputFile:
            outputFieldnames = [ 'time',
                                'target1_x', 'target1_y', 'target1_z', \
                                'target2_x', 'target2_y', 'target2_z', \
                                'lasers_yaw', \
                                'local_vel_x', 'local_vel_y', 'local_vel_z', 'local_vel_yaw', \
                                'filtered_x', 'filtered_y', 'filtered_z', 'filtered_yaw']

            fieldnames = ['time', 'setpoint_x', 'setpoint_y', 'setpoint_z', 'setpoint_yaw', \
                      'local_x', 'local_y', 'local_z', 'local_roll', 'local_pitch', 'local_yaw', \
                      'lasers_x', 'lasers_y', 'lasers_z', 'lasers_yaw', \
                      'raw_x_1', 'raw_x_2', 'raw_y_1', 'raw_y_2','raw_z_1', 'raw_z_2', \
                      'local_vel_x', 'local_vel_y', 'local_vel_z', 'local_vel_yaw', \
                      'accel_lin_x', 'accel_lin_y', 'accel_lin_z', \
                      'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', \
                      'filtered_x', 'filtered_y', 'filtered_z', 'filtered_yaw', \
                      'accel_no_gravity_x', 'accel_no_gravity_y', 'accel_no_gravity_z', \
                      'Xk_x', 'Xk_y', 'Xk_z', 'Xk_yaw',  \
                      'K_x', 'K_y', 'K_z', 'K_yaw',  \
                      'Xkp_x', 'Xkp_y', 'Xkp_z', 'Xkp_yaw', \
                      'target1_x', 'target1_y', 'target1_z', \
                      'target2_x', 'target2_y', 'target2_z', \
                      'isArmed?', 'information_utilisateur']

            data_reader = csv.DictReader(inputFile, fieldnames=fieldnames, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            data_writer = csv.writer(outputFile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

            FilterX = simple_filter()
            FilterY = simple_filter()
            FilterZ = simple_filter()
            FilterYaw = simple_filter()
            lasttime = 0.0
            count = 0
            for row in data_reader : 
                if(count == 2):
                    data_writer.writerow(outputFieldnames)
                if(count > 6):
                    # target = (X1, X2, Y1, Y2, Z1, Z2, Yaw1, Yaw2) in m/s and rad
                    # linearSpeed = (Vx, Vy, Vz) in m/s
                    # LinearAcceleration = (Ax, Ay, Az) in m/s/s
                    # AngularSpeed = yawRate in rad/s
                    # dt in seconds
                    # Kalman.next(target, linearSpeed, linearAcceleration, angularSpeed, dt)
                    Measures =  [[float(row['target1_x']), float(row['target2_x'])], \
                                [float(row['target1_y']), float(row['target2_y'])], \
                                [float(row['target1_z']), float(row['target2_z'])], \
                                float(row['lasers_yaw'])]

                    linearSpeed = [ float(row['local_vel_x']), \
                                    float(row['local_vel_y']), \
                                    float(row['local_vel_z'])]

                    angularSpeed = [float(row['local_vel_yaw'])]

                    dt = float(row['time']) - lasttime

                    X = FilterX.next(Measures[0], dt, linearSpeed[0])
                    Y = FilterY.next(Measures[1], dt, linearSpeed[1])
                    Z = FilterZ.next(Measures[2], dt, linearSpeed[2])
                    Yaw = FilterYaw.next(Measures[3], dt, angularSpeed[0])
                    
                    lasttime = float(row['time'])

                    data_writer.writerow([row['time'], row['target1_x'], row['target1_y'], row['target1_z'], \
                                row['target2_x'], row['target2_y'], row['target2_z'], \
                                row['lasers_yaw'], \
                                row['local_vel_x'], row['local_vel_y'], row['local_vel_z'], row['local_vel_yaw'], \
                                str(X), str(Y), str(Z), str(Yaw)])

                count = count +1

testSimpleFilter()