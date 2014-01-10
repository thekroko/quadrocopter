'''
gyro calibration functions 01

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.
'''

import sys
import string
import serial
from struct import unpack, pack
from optparse import OptionParser
import time

import sched
from sys import stdout
import matplotlib.pyplot as plt
import numpy as np

#serial port
SERIALPORT = "/dev/ttyUSB0"
SERIALBAUD = 38400

#serial command to get data
SERGETCOMMAND = '\x21'

#number of read to take for the avarage value
READAVARAGETOT = 500

#motion degrees for scale factor estimation
DEGMOTION = 90

#repeat the reading of the angular velocity every n seconds
DEGSCHEDULETIME = 0.05

#motion threshold percent to calculate motion values
MOTIONTHRESHOLDPERCENT = 1.5

#number of samples to take for temperature compansation
TEMPCOMPSAMPLES = 1000


#flush input if present
def ser_flush():
	while ser.inWaiting():
		ser.read()
			
#read from serial until the character specified and number of char has reached
def ser_readuntil(untilchar, numchars):
	buf = ""
	bufnum = 0
	done = 0
	while not done:
		#read from serial
		n = ser.read()
		buf = buf + n
		bufnum = bufnum + 1
		#check if this could be the last char
		if n == untilchar:
			#check if the number of char readed is correct
			if bufnum == numchars:
				done = 1
			else:
				done = 1
				buf = ""
				bufnum = 0
	return buf
	
#read data from chip
def readdata(avaragesamples):
	dataret = [0, 0, 0]
	if avaragesamples > 0:
		datatot = [0, 0, 0]
		dataread = [0, 0, 0]
		#repat read avaragesamples times and avarage values
		for x in range(avaragesamples):
			#send the get values command
			ser.write(SERGETCOMMAND)
			#read values
			line = ser_readuntil('\n', 7)
			i = 1
			datanum = 0
			while i < len(line):
				#combine back the read values
				num = unpack('<h', line[(i-1)] + line[(i-1)+1])
				dataread[datanum] = int(num[0])
				datanum = datanum + 1
				i = i + 2
				if i == len(line):
					break
			datatot[0] = datatot[0] + dataread[0]
			datatot[1] = datatot[1] + dataread[1]
			datatot[2] = datatot[2] + dataread[2]
		dataret[0] = float(datatot[0]) / avaragesamples
		dataret[1] = float(datatot[1]) / avaragesamples
		dataret[2] = float(datatot[2]) / avaragesamples
	else:
		#send the get values command
		ser.write(SERGETCOMMAND)
		#read values
		line = ser_readuntil('\n', 7)
		i = 1
		datanum = 0
		while i < len(line):
			#combine back the read values
			num = unpack('<h', line[(i-1)] + line[(i-1)+1])
			dataret[datanum] = int(num[0])
			datanum = datanum + 1
			i = i + 2
			if i == len(line):
				break
	return dataret
	
#read bias raw values
def readbias():
	ser_flush()
	data = [0, 0, 0]
	readval = 0
	#repeat until user confirm read values
	while not readval:
		raw_input("leave gyro in a stable position to obtain bias, press any key to continue ")
		#read data
		data = readdata(READAVARAGETOT)
		print "  read %.2f %.2f %.2f " % (data[0], data[1], data[2])
		#check if repeat, or continue to next reading 
		while 1:
			readkey = raw_input("press C to continue, R to repeat the reading: ")
			if readkey == "C" or readkey == "c":
				readval = 1
				break
			elif readkey == "R" or readkey == "r":
				break
	return data

#read an angular turn
readangular_datasum = 0	#readangular global variable
def readangular(index, offset, motionval, ratationplane, rotationtext):
	data = [0, 0, 0]
	retinteg = 0
	readval = 0
	readnum = 0
	#repeat until user confirm read values
	while not readval:
		ser_flush()
			
		inputstring = "put gyro such as " + ratationplane + " plane is parallel to the " + ratationplane + " rotation plane, the rotate it " + rotationtext + " for " + str(DEGMOTION) + " degrees to read rotation velocity, reading will start when gyro begin moved, press any key to continue "
		raw_input(inputstring)
			
		#check for the motion start
		moving = 0
		while not moving:
			data = readdata(0)
			#start reading when we move
			if abs(data[index]) > motionval:
				moving = 1
		print "  reading started..."
		
		#read and sum raw values, integrating it
		datasum = 0
		s = sched.scheduler(time.time, time.sleep)
		def do_readandintegrate(s,sdatasum,smotioncount,stimepre): 
			data = readdata(0)
			#print smotioncount , " "  , data
			moving = 1
			num = data[index]
			#check for the motion stop under a threshold
			if abs(num) < motionval:
				if smotioncount > 10:
					#if we are here more than n time we are realy stopped
					moving = 0
				else:
					smotioncount = smotioncount + 1
			else:
				#reset motion count
				smotioncount = 0
			#add values to sum
			#print "%d %d %d %d %.2f" % (sdatasum, num, index, offset, time.time()-stimepre)
			num = (num + offset) * (time.time()-stimepre) #integrate values using previos time
			sdatasum = sdatasum + num
			if moving == 1:
				#if we are moving do one reading more
				s.enter(DEGSCHEDULETIME, 10, do_readandintegrate, (s,sdatasum,smotioncount,time.time()))
			else:
				#if we are not moving leave this shedule
				global readangular_datasum
				readangular_datasum = sdatasum
		s.enter(DEGSCHEDULETIME, 10, do_readandintegrate, (s,0,0,time.time()))
		s.run()
		print "  reading stopped"
		
		#assign integrated sum
		datasum = readangular_datasum
		#estimate scale for rotation degrees
		if datasum != 0:
			retinteg = DEGMOTION / datasum
		else:
			retinteg = 0
		print "  sum is %.2f, estimated scale factor for the rotation of %d is %.4f" % (datasum, DEGMOTION, retinteg)
		
		#check if repeat, or continue to next reading 
		while 1:
			readkey = raw_input("press C to continue, R to repeat the reading: ")
			if readkey == "C" or readkey == "c":
				readval = 1
				break
			elif readkey == "R" or readkey == "r":
				break
	return retinteg

#read temperature compensation
def readtempcomp():
	ser_flush()
	data = [0, 0, 0]
	dataret = [0, 0, 0]
	datareg = [0, 0, 0]
	datareg[0] = []
	datareg[1] = []
	datareg[2] = []
	readval = 0
	collectedsamples = 0
	#repeat until user confirm read values
	while not readval:
		collectedsamples = 0
		#collect values
		raw_input("to obtain temperature compensation values, leave gyro in a stable position, cool the gyro, then let the script collect " + str(TEMPCOMPSAMPLES) + " values while the gyro return to ambient temperature, press any key to continue ")
		while collectedsamples < TEMPCOMPSAMPLES:
			#read data
			data = readdata(50)
			datareg[0].append([data[0], collectedsamples])
			datareg[1].append([data[1], collectedsamples])
			datareg[2].append([data[2], collectedsamples])
			collectedsamples = collectedsamples + 1
			stdout.write("\r  read %4d samples ( get %4d, %4d, %4d )" % (collectedsamples, data[0], data[1], data[2]))
			stdout.flush()
			time.sleep(0.1) #give the chip time to return to ambient temperature
		stdout.write("\n")
		stdout.flush()
		
		#estimate temperature compansation values
		for index in range(0,3):
			x = []
			y = []
			yfit = []
			for i in datareg[index]:
				y.append(float(i[0]))
				x.append(int(i[1]))			
			#do linear regression
			A = np.vstack([x, np.ones(len(x))]).T
			m, c = np.linalg.lstsq(A, y)[0]
			#assign result value
			dataret[index] = m
			#built fitted y
			for i in datareg[index]:
				yfit.append(int(i[1]) * m + c)
			#plot table
			plt.title('Linear Regression')
			plt.plot(x, y, 'o', label='Original data', markersize=3)
			plt.plot(x, yfit, 'r', label='Fitted line')
			plt.show()

		#show result
		print "temp compensation values %.6f %.6f %.6f " % (dataret[0], dataret[1], dataret[2])
					
		#check if repeat, or continue to next reading 
		while 1:
			readkey = raw_input("press C to continue, R to repeat the reading: ")
			if readkey == "C" or readkey == "c":
				readval = 1
				break
			elif readkey == "R" or readkey == "r":
				break
	return dataret

#main function			
if __name__ == '__main__':
	print "Gyroscope Calibration functions 01"
	print ""
	
	parser = OptionParser()
	parser.add_option("-d", "--debug", dest="opt_debug", action="store_true", help="debug output")
	parser.add_option("-c", "--calib", dest="opt_calib", action="store_true", help="run calibration routine")
	parser.add_option("-t", "--tempcomp", dest="opt_tempcomp", action="store_true", help="run temperature componesation calibration routine")
	(options, args) = parser.parse_args()
	
	if options.opt_debug or options.opt_calib or options.opt_tempcomp:
	
		#set serial port
		ser=serial.Serial(port=SERIALPORT, baudrate=SERIALBAUD , bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
		ser.close()
		ser.open()
		
		if ser.isOpen():
			print ("Port " + ser.portstr + " opened")
			
			if options.opt_debug:
				print "Debug output"
				
				while 1:
					data = readdata(0)
					print "  read %d %d %d" % (data[0], data[1], data[2])
					time.sleep(0.5)
			
			elif options.opt_calib:
			
				while 1:
					#read bias values
					bias = readbias()
										
					#calculate motion values as 5% more than bias values
					minval_x = int(abs(bias[0]) + MOTIONTHRESHOLDPERCENT*abs(bias[0]))
					minval_y = int(abs(bias[1]) + MOTIONTHRESHOLDPERCENT*abs(bias[1]))
					minval_z = int(abs(bias[2]) + MOTIONTHRESHOLDPERCENT*abs(bias[2]))
					print "  motion values %.2f %.2f %.2f " % (minval_x, minval_y, minval_z)
										
					#read an agnular velocity
					readnum = 0
					
					Xc = readangular(0, -bias[0], minval_x, "X", "clockwise")
					Xa = readangular(0, -bias[0], minval_x, "X", "anticlockwise")
					Yc = readangular(1, -bias[1], minval_y, "Y", "clockwise")
					Ya = readangular(1, -bias[1], minval_y, "Y", "anticlockwise")
					Zc = readangular(2, -bias[2], minval_z, "Z", "clockwise")
					Za = readangular(2, -bias[2], minval_z, "Z", "anticlockwise")
					
					#calculate the calibration values
					bias_X = bias[0]
					bias_Y = bias[1]
					bias_Z = bias[2]
					gain_X = (abs(Xc) + abs(Xa))/2
					gain_Y = (abs(Yc) + abs(Ya))/2
					gain_Z = (abs(Zc) + abs(Za))/2
					
					#print out values
					print ""
					print "read values:"
					print "  bias for X %.2f" % bias_X
					print "  bias for Y %.2f" % bias_Y
					print "  bias for Z %.2f" % bias_Z
					print "  scale factor for X clockwise is %.4f" % Xc
					print "  scale factor for X anticlockwise is %.4f" % Xa
					print "  scale factor for Y clockwise is %.4f" % Yc
					print "  scale factor for Y anticlockwise is %.4f" % Ya
					print "  scale factor for Z clockwise is %.4f" % Zc
					print "  scale factor for Z anticlockwise is %.4f" % Za
					print "suggested calibration values"
					print "  offset X %.2f " % bias_X
					print "  offset Y %.2f " % bias_Y
					print "  offset Z %.2f " % bias_Z
					print "  scale fator X %.4f " % gain_X
					print "  scale fator Y %.4f " % gain_Y
					print "  scale fator Z %.4f " % gain_Z
					
					print ""
					readkey = raw_input("press any key to repeat reading, or X to exit ")
					if readkey == "X" or readkey == "x":
						break
				
			elif options.opt_tempcomp:
				while 1:
					
					#read temperature compensation
					tempcomp = readtempcomp()
										
					#print out values
					print ""
					print "read values:"
					print "  temp compensation X %.6f " % tempcomp[0]
					print "  temp compensation Y %.6f " % tempcomp[1]
					print "  temp compensation Z %.6f " % tempcomp[2]
					
					print ""
					readkey = raw_input("press any key to repeat reading, or X to exit ")
					if readkey == "X" or readkey == "x":
						break
						
				
			ser.close()
		else:
			print ("Can not oper port")
		
	else:
		parser.print_help()
		
	
