# Import system modules
import subprocess
import os
import time
from shutil import copyfile
from distutils.dir_util import copy_tree
import datetime
import smtplib
import ctypes  # MessageBox


def sendEmail(i):
	if i==0:
		#send to Lirong
		fromaddr = 'xiang@iastate.edu'
		toaddrs  = 'xiang@iastate.edu'
		msg = "\r\n".join([
			"From: xiang@iastate.edu",
			"To: xiang@iastate.edu",
			"Subject: The Robot Stopped!!!",
			"",
			"This is an automatically sent email.\n\nThe robot have stopped, may need user interaction\n\nIf the camera is above the balance, click OK, check the output information, and press Enter to continue.\n\n If an error message is thrown on the UR10 screen, restart the UR10 and rerun the Python script.\n\nElse, please text Lirong (xiang@iastate.edu, 5157153757)."
			])
		username = 'xiang@iastate.edu'
		password = 'Xlr@036428'
		server = smtplib.SMTP('smtp.gmail.com:587')
		server.ehlo()
		server.starttls()
		server.login(username,password)
		server.sendmail(fromaddr, toaddrs, msg)
		server.quit()	
		#send to Trevor
		fromaddr = 'xiang@iastate.edu'
		toaddrs  = 'tmnolan@iastate.edu'
		msg = "\r\n".join([
			"From: xiang@iastate.edu",
			"To: tmnolan@iastate.edu",
			"Subject: The Robot Stopped!!!",
			"",
			"This is an automatically sent email.\n\nThe robot have stopped, may need user interaction\n\nIf the camera is above the balance, click OK, check the output information, and press Enter to continue.\n\n If an error message is thrown on the UR10 screen, restart the UR10 and rerun the Python script.\n\nElse, please text Lirong (xiang@iastate.edu, 5157153757)."
			])
		username = 'xiang@iastate.edu'
		password = 'Xlr@036428'
		server = smtplib.SMTP('smtp.gmail.com:587')
		server.ehlo()
		server.starttls()
		server.login(username,password)
		server.sendmail(fromaddr, toaddrs, msg)
		server.quit()
	if i==1:
		fromaddr = 'xiang@iastate.edu'
		toaddrs  = 'xiang@iastate.edu'
		msg = "\r\n".join([
			"From: xiang@iastate.edu",
			 "To: xiang@iastate.edu",
			 "Subject: The Robot Finished!!!",
			 "",
			 "This is an automatically sent email.\n\nThe robot have finished work today.\n\nThe data will be backed up to RoAdData/experiment5/\n\nNext run time: 9 am"
			  ])
		username = 'xiang@iastate.edu'
		password = 'Xlr@036428'
		server = smtplib.SMTP('smtp.gmail.com:587')
		server.ehlo()
		server.starttls()
		server.login(username,password)
		server.sendmail(fromaddr, toaddrs, msg)
		server.quit()
		#send to Trevor
		fromaddr = 'xiang@iastate.edu'
		toaddrs  = 'tmnolan@iastate.edu'
		msg = "\r\n".join([
			"From: xiang@iastate.edu",
			 "To: tmnolan@iastate.edu",
			 "Subject: The Robot Finished!!!",
			 "",
			 "This is an automatically sent email.\n\nThe robot have finished work today.\n\nThe data will be backed up to RoAdData/experiment5/\n\nNext run time: 9 am"
			  ])
		username = 'xiang@iastate.edu'
		password = 'Xlr@036428'
		server = smtplib.SMTP('smtp.gmail.com:587')
		server.ehlo()
		server.starttls()
		server.login(username,password)
		server.sendmail(fromaddr, toaddrs, msg)
		server.quit()	
	return
	

#main
firstTime=0

MessageBox=ctypes.windll.user32.MessageBoxA(0, 'Do you want the robot to start working now?','RoAD', 1)
if MessageBox==1:
	firstTime=1



while True:
	now = datetime.datetime.now()
	print now
	print "next run time, 9:00 am"

	if now.hour==9 and now.minute==0 or firstTime==1: 
		firstTime=0
		while True:
			proc=subprocess.Popen("C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/RoAD2.0.exe")

			last_pot_id_=0;
			count=0;

			time.sleep(30)
			while True:
				
				program_status=proc.poll()

				if program_status==None:
					print "*****************the program is running*****************"
				else:
					print "*****************the program finished*******************"
					break

				time.sleep(30)

				#check if the robot has stopped
				copyfile("currentStatusFile.txt", "copy.txt")
				file=open("copy.txt","r")

				for line in file:
					id_array=line.split(",")
					current_pot_id_=id_array[1]
					break;

				if current_pot_id_==last_pot_id_:
					count=count+1
				else:
					count=0
					last_pot_id_=current_pot_id_

				if count>8:  #more than 300 sec
					print "time out caught, restart"
					proc.kill()
					#send an email
					sendEmail(0)
					#wait for user input
					while True:    # infinite loop
						MessageBox=ctypes.windll.user32.MessageBoxA(0, 'Click OK to Continue','RoAD', 1)
						if MessageBox==1:
							print "ready to restart"
							break

			#one section finished
			print "****one section finished, ready to go next position*****"
			
			#check if all the pots finished

			copyfile("currentStatusFile.txt", "copy.txt")
			file=open("copy.txt","r")

			for line in file:
				id_array=line.split(",")
				current_pot_id_=int(id_array[1])
				break

			print current_pot_id_

			if current_pot_id_==239:  #####to be tested
				print "all the pots finished!"
				time.sleep(10)
				break

			time.sleep(10)

		print "finish all the pots today, start to back up data"
		folder_name=str(now.year)+"-"+str(now.month)+"-"+str(now.day)
		fromDirectory = "C:/RoAdData/" +folder_name
		toDirectory = "C:/Users/phenobot/Box/RoAdData/experiment5/" + folder_name
		
		copy_tree(fromDirectory, toDirectory)	
		#send a email
		sendEmail(1)
	else:
		if now.hour>14 or now.hour<8:
			print "sleep during 15 pm to 7 am\n"
			time.sleep(60*60)
		else:
			time.sleep(60)