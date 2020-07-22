from Kalman import KalmanAngle
from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Gyroscope import *
import time
import math
import sys


class PhidgetKalman(object):
	#Read the gyro and acceleromater values from MPU6050
	def __init__(self):
		self.kalmanX = KalmanAngle()
		self.kalmanY = KalmanAngle()

		self.RestrictPitch = True	#Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
		self.radToDeg = 57.2957786
		self.kalAngleX = 0
		self.kalAngleY = 0
		try:
			self.accel = Accelerometer() #ch.getAcceleration()
			self.gyro  = Gyroscope()	  #ch.getAngularRate()
		except Exception as e:
			print("Cannot initalize, try checking you have the Phidget22 library installed.")
			print("Error:", e)
			sys.exit()
		# return self.accel, self.gyro
	
	def start(self):
		try:
			self.accel.openWaitForAttachment(1000)
			self.gyro.openWaitForAttachment(1000)
		except Exception as e:
			print("Issue with attaching to IMU. The IMU maybe connected to something else right now..")
			print("Error:", e)
			sys.exit()
	def stop(self):
		self.accel.close()
		self.gyro.close()
		print("Stopping")
	def get_angles(self, measure_time=5):
		#Keep going
		time.sleep(1)
		accX, accY, accZ = self.accel.getAcceleration()
		if (self.RestrictPitch):
			roll = math.atan2(accY,accZ) * self.radToDeg
			pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * self.radToDeg
		else:
			roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * self.radToDeg
			pitch = math.atan2(-accX,accZ) * self.radToDeg
		# print(roll)
		self.kalmanX.setAngle(roll)
		self.kalmanX.setAngle(pitch)
		gyroXAngle = roll;
		gyroYAngle = pitch;
		compAngleX = roll;
		compAngleY = pitch;

		timer = time.time()
		compare_timer = time.time()
		flag = 0
		try:
			while int(time.time()) < int(measure_time) + int(compare_timer): #Should only run for about 5 seconds <- Could make lower
			
				#Read Accelerometer raw value
				accX, accY, accZ = self.accel.getAcceleration()

				#Read Gyroscope raw value
				gyroX, gyroY, gyroZ = self.gyro.getAngularRate()


				dt = time.time() - timer
				timer = time.time()

				if (self.RestrictPitch):
					roll = math.atan2(accY,accZ) * self.radToDeg
					pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * self.radToDeg
				else:
					roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * self.radToDeg
					pitch = math.atan2(-accX,accZ) * self.radToDeg

				gyroXRate = gyroX/131
				gyroYRate = gyroY/131

				if (self.RestrictPitch):

					if((roll < -90 and self.kalAngleX >90) or (roll > 90 and self.kalAngleX < -90)):
						self.kalmanX.setAngle(roll)
						complAngleX = roll
						self.kalAngleX   = roll
						gyroXAngle  = roll
					else:
						self.kalAngleX = self.kalmanX.getAngle(roll,gyroXRate,dt)

					if(abs(self.kalAngleX)>90):
						gyroYRate  = -gyroYRate
						self.kalAngleY  = self.kalmanX.getAngle(pitch,gyroYRate,dt)
				else:

					if((pitch < -90 and self.kalAngleY >90) or (pitch > 90 and self.kalAngleY < -90)):
						self.kalmanX.setAngle(pitch)
						complAngleY = pitch
						self.kalAngleY   = pitch
						gyroYAngle  = pitch
					else:
						self.kalAngleY = self.kalmanX.getAngle(pitch,gyroYRate,dt)

					if(abs(self.kalAngleY)>90):
						gyroXRate  = -gyroXRate
						self.kalAngleX = self.kalmanX.getAngle(roll,gyroXRate,dt)

				#angle = (rate of change of angle) * change in time
				gyroXAngle = gyroXRate * dt
				gyroYAngle = gyroYAngle * dt

				#compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
				compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
				compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

				if ((gyroXAngle < -180) or (gyroXAngle > 180)):
					gyroXAngle = self.kalAngleX
				if ((gyroYAngle < -180) or (gyroYAngle > 180)):
					gyroYAngle = self.kalAngleY

				self.X_angle = self.kalAngleX
				self.Y_angle = self.kalAngleY
				#print("Angle X: " + str(self.kalAngleX)+"   " +"Angle Y: " + str(self.kalAngleY))
				#print(str(roll)+"  "+str(gyroXAngle)+"  "+str(compAngleX)+"  "+str(self.kalAngleX)+"  "+str(pitch)+"  "+str(gyroYAngle)+"  "+str(compAngleY)+"  "+str(self.kalAngleY))
				time.sleep(0.005)

		except KeyboardInterrupt:
			print("test")
		return self.X_angle, self.Y_angle
	

if __name__ == "__main__":
	obj = PhidgetKalman()
	obj.start()
	print(obj.get_angles(measure_time=5)) #5 seconds seems to be a good amount of time. 4 is okay. Anything lower doesn't work well.
	obj.stop()
	print('End')