#!/usr/bin/python
import ms5837
import time
import rospy
from std_msgs.msg import Float32

sensor = ms5837.MS5837_30BA() 

'''
print("Pressure: %.2f psi") % (
sensor.pressure(ms5837.UNITS_psi))

print("Temperature: %.2f K") % (
,
sensor.temperature(ms5837.UNITS_Kelvin))

freshwaterDepth = sensor.depth() # default is freshwater
sensor.setFluidDensity(1000) # kg/m^3



print("MSL Relative Altitude: %.2f m") % sensor.altitude() # relative to Mean Sea Level pressure in air

time.sleep(5)

# Spew readings
        if sensor.read():
            
        else:
                print "Sensor read failed!"
                exit(1)
'''




def spewDepth():
    pub  = rospy.Publisher('depth', Float32, queue_size = 1)
    rospy.init_node('sensor_Depth', anonymous= True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        read_depth = sensor.depth() # freshwater depth
        rospy.loginfo(read_depth)
        pub.publish(read_depth)
        rate.sleep()

def spewPressure():
    pub2  = rospy.Publisher('pressure', Float32, queue_size = 1)
    rospy.init_node('sensor_Pressure', anonymous= True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        read_pressure = sensor.pressure(ms5837.UNITS_psi) # pressure in PSI
        rospy.loginfo(read_pressure)
        pub2.publish(read_pressure)
        rate.sleep()






if __name__ == '__main__':
    # We must initialize the sensor before reading it
    if not sensor.init():
        print "Sensor could not be initialized"
        exit(1)

# We have to read values from sensor to update pressure and temperature
    if not sensor.read():
        print "Sensor read failed!"
        exit(1)

    try:
        spewDepth()
        spewPressure()
    except rospy.ROSInterruptException:
        pass

