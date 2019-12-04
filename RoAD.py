import sys
from Tkinter import *
import tkFileDialog as filedialog
import tkMessageBox
#from Tkinter import Filedialog
#from Tkinter import Messagebox
from random import shuffle
import csv

# Import system modules
import subprocess
import os
import time
from shutil import copyfile
from distutils.dir_util import copy_tree
import datetime
import smtplib #email
import ctypes  # MessageBox

import smtplib, ssl
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText


str_list=["genotype: ","replication: ", "shuffle_within_replication: ","grams_of_water: ","grams_of_pcz: ","soil_weight: ","pot_weight: ", "block_within_replication: "]

myopen=""

def mOpen():
	global myopen
	myopen=filedialog.askopenfilename(  filetypes=(("text files",".txt"),("All files","*.*")))
	file=open(myopen,"r")
	text_input.insert(END,'Input:\n\n','big')
	i=0;
	for line in file:
		text_input.insert(END,str_list[i],'big')	
		line=line.replace(" ","")
		temp=line.split(":")
		text_input.insert(END,temp[1]+"\n")	
		i=i+1
	return

def mQuit():
	mExit=tkMessageBox.askokcancel(title="Quit",message="Are You Sure?")
	if mExit>0:
		mGui.destroy()
		return
def mAbout():
	tkMessageBox.showinfo(title="About",message="1. Click 'File' to import a text file.\n2.Click 'Run' to get the results.\n3.Click 'Save' to save the results.\n\nFor the text file, please put plant_type, replication, shuffle_within_replication (true or false), target_weight_water and target_weight_Hormane seperatedd by commas in 5 lines.\n\nFor example:\n\nPOT,WT,56,TRT,CNT\n6\nfalse\n40,60,80,100\n50,70")
	return
def mRun():
	file=open(myopen,"r")
	array=[]
	
	for line in file:
		line=line.replace(" ","")
		temp=line.split(":")
		array.append(temp[1])

	G=array[0].split(",")
	R=array[1].split(",")
	S=array[2].split(",")
	grams_of_water=array[3].split(",")
	grams_of_pcz=array[4].split(",")

	tmp=array[5].split(",")
	soil_weight=float(tmp[0].replace("\n",""))
	tmp=array[6].split(",")
	pot_weight=float(tmp[0].replace("\n",""))

	tmp=array[7].split(",")
	block_within_replication=tmp[0].replace("\n","")

	W=[]
	treatmentAll=[]
	for i in range(0,len(grams_of_water)):
		weight_tmp=soil_weight*float(grams_of_water[i])+soil_weight+pot_weight
		W.append(round(weight_tmp,2))
		treatmentAll.append("W,"+str(round(weight_tmp,2)))
	H=[]
	#################
	for i in range(0,len(grams_of_pcz)):
		if float(grams_of_pcz[i])>0:
		 	weight_tmp=soil_weight*float(grams_of_pcz[i])+pot_weight+soil_weight
		 	H.append(round(weight_tmp,2))
		 	treatmentAll.append("H,"+str(round(weight_tmp,2)))

	#print treatmentAll

	global final_list
	final_list=[]

	if S[0].replace("\n","") in ["true", "True", "TRUE"]:
		for i in range(1,int(R[0])+1):

			if block_within_replication in ["true", "True", "TRUE"]:

				temp_list=[]

				shuffle(G)

				for g in G:

					if str(g)=="\n":
						continue

					genotype_id=str(g).replace("\n","")

					shuffle(treatmentAll)

					for t in treatmentAll:
						if str(t)=="\n":
							continue

						treatment=str(t).replace("\n","")

						s=genotype_id+","+treatment+","+str(i)

						temp_list.append(s)
						
				final_list+=temp_list


			else:
				temp_list=[]

				for g in G:

					if str(g)=="\n":
						continue

					genotype_id=str(g).replace("\n","")

					for w in W:
						if str(w)=="\n":
							continue

						water_level=str(w).replace("\n","")

						s=genotype_id+",W,"+water_level+","+str(i)
						temp_list.append(s)

					if len(H)>0:
						for h in H:
							if str(h)=="\n":
								continue

							hormone_level=str(h).replace("\n","")

							s=genotype_id+",H,"+hormone_level+","+str(i)
							temp_list.append(s)

				shuffle(temp_list)
				final_list+=temp_list

	else:
		for g in G:

			if str(g)=="\n":
				continue

			genotype_id=str(g).replace("\n","")

			for w in W:
				if str(w)=="\n":
					continue

				water_level=str(w).replace("\n","")
				for i in range(1,int(R[0])+1):
					s=genotype_id+",W,"+water_level+","+str(i)
					final_list.append(s)
			if len(H)>0:
				for h in H:
					if str(h)=="\n":
						continue

					hormone_level=str(h).replace("\n","")
					for i in range(1,int(R[0])+1):
						s=genotype_id+",H,"+hormone_level+","+str(i)
						final_list.append(s)
		shuffle(final_list)	


	sample_size=len(final_list);

	if(sample_size>240):
		tkMessageBox.showinfo(title="Warning",message="Sample_size > 240, please reload a file")
	else:
		text_output.insert(END,'Sample Size: ','big')
		text_output.insert(END,str(sample_size)+"\n")

	for g in final_list:
		text_output.insert(END,g+"\n")

	return

def mSave():

	save_path=filedialog.asksaveasfilename(initialdir = "/",title = "Select file",filetypes = (("text files","*.txt"),("csv files","*.csv"),("all files","*.*")))
	final_list.insert(0,"geno,treatment,level,rp")
#	out = csv.writer(open(save_path+".csv","w"), delimiter='\n',quoting=csv.QUOTE_ALL)
#	out.writerow(final_list)
	file = open(save_path+".txt","w")
	for i in range(0, len(final_list)):
		file.write(final_list[i])
		file.write("\n")
	file.close()
	return

def rClear():
	text_output.delete(1.0,END)
	return;
def fClear():
	text_input.delete(1.0,END)
	text_output.delete(1.0,END)


stopBox=[]
stopBox.append("Subject: The Robot Stopped!!!")
stopBox.append("This is an automatically sent email.\n\nThe robot have stopped, may need user interaction\n\nPlease check the RoAD manual Google doc.")

finshBox=[]
finshBox.append("Subject: The Robot Finished!!!")
finshBox.append("This is an automatically sent email.\n\nThe robot have finished work today.\n\nThe data will be backed up to RoAdData/\n\nNext run time: 9 am")


emailBox=["lr_xiang@hotmail.com","jmelmore@iastate.edu","pingw@iastate.edu"]

def sendEmail(emailID, finished):
		#send to Lirong
		fromaddr = 'xiang@iastate.edu'
		toaddrs=emailBox[emailID]
		str="To: "+toaddrs

		if finished:
			subject=finshBox[0]
			content=finshBox[1]
		else:
			subject=stopBox[0]
			content=stopBox[1]

		msg = "\r\n".join([
			"From: isu.enviratron@gmail.com",
			#"To: xiang@iastate.edu",
			str,
			subject,
			"",
			content
			])
		username = 'isu.enviratron@gmail.com'
		password = 'abetang123'
		server = smtplib.SMTP('smtp.gmail.com:587')
		server.ehlo()
		server.starttls()
		server.login(username,password)
		server.sendmail(fromaddr, toaddrs, msg)
		server.quit()	

def notify_managers(finished): #doen't work with python 2.7
   
    port = 587  # For starttls
    password = 'abetang123'

    # Create a secure SSL context
    context = ssl.create_default_context()



    with smtplib.SMTP("smtp.gmail.com", port) as server:
        server.starttls(context=context)
        server.login("isu.enviratron@gmail.com", password)
       
        # TODO: Send email here
        sender_email = "isu.enviratron@gmail.com"
        receiver_email = ['jmelmore@iastate.edu','xiang@iastate.edu','pingw@iastate.edu']

        if finished:
                msg['Subject']=finshBox[0]
                body=finshBox[1]
        else:
                msg['Subject']=stopBox[0]
                body=stopBox[1]

        # body = 'Remote desktop and check log_file.txt'
        msg = MIMEMultipart()

        # msg['Subject'] = 'Rover program stopped!'
        msg['From'] = sender_email
        msg['To'] = (', ').join(receiver_email)

        msg.attach(MIMEText(body,'plain'))
        server.send_message(msg)

def RoAdWatcher():
	
	global label_file_path;
	if label_file_path=="":
		MessageBox=ctypes.windll.user32.MessageBoxA(0, 'Please select the label file','RoAD Warning', 0)
		return

	firstTime=0
	MessageBox=ctypes.windll.user32.MessageBoxA(0, 'Do you want the robot to start working now?','RoAD', 1)
	if MessageBox==1:
		firstTime=1

	pot_map_size = 240;
	while True:
		now = datetime.datetime.now()
		print now
		print "next run time, 9:00 am"

		if now.hour==9 and now.minute==0 or firstTime==1: 
			firstTime=0
			while True:
				
				#proc=subprocess.Popen("C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/RoAD2.0.exe")

				args = "C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/PickPots.exe " + label_file_path 
				print args
				#proc=subprocess.call(args)     #, shell=False
				proc=subprocess.Popen(args)

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

					if count>10:  #more than 300 sec
						print "time out caught, restart"
						proc.kill()
						#send an email
						for i in range(len(emailBox)):
                                                 	sendEmail(i, 0)
						
						#wait for user input
						while True:    # infinite loop
							MessageBox=ctypes.windll.user32.MessageBoxA(0, 'Click OK to Continue','RoAD', 1)
							if MessageBox==1:
								print "ready to restart"
								break

				#one section finished
				#print "****one section finished, ready to go next position*****"
				
				#check if all the pots finished

				copyfile("currentStatusFile.txt", "copy.txt")
				file=open("copy.txt","r")

				for line in file:
					id_array=line.split(",")
					current_pot_id_=int(id_array[1])
					break

				print current_pot_id_

				if current_pot_id_== (pot_map_size -1):  #####to be tested
					print "all the pots finished!"
					time.sleep(10)
					break

				time.sleep(10)

			print "finish all the pots today, start to back up data"
			folder_name=str(now.year)+"-"+str(now.month)+"-"+str(now.day)
			fromDirectory = "C:/RoAdData/" +folder_name
			toDirectory = "C:/Users/phenobot/Box/RoAdData/experiment13/" + folder_name
			
			copy_tree(fromDirectory, toDirectory)	

			##write to day_count
			fh = open('day_count.txt', "r")
			lineList = fh.readlines()
			fh.close()
			current_day = lineList[-1]
			fh = open('day_count.txt', "a")
			fh.write("\n" + str(int(current_day)+1))
			fh.close()
			#write end

			#send a email
			for i in range(len(emailBox)):
				sendEmail(i, 1)
#			notify_managers(1)
		else:
			if now.hour>14 or now.hour<8:
				print "sleep during 15 pm to 7 am\n"
				time.sleep(60*60)
			else:
				time.sleep(60)

def loadFile():
	global label_file_path
	label_file_path=filedialog.askopenfilename(filetypes = (("text files","*.txt"),("csv files","*.csv"),("all files","*.*")))
	print label_file_path

def placePots():
	global label_file_path
	if label_file_path=="":
		MessageBox=ctypes.windll.user32.MessageBoxA(0, 'Please select the label file','RoAD Warning', 0)
		return
	#proc=subprocess.Popen("C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/PlacePots.exe")###

	#FNULL = open(os.devnull, 'w')    #use this if you want to suppress output to stdout from the subprocess
	args = "C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/PlacePots.exe " + label_file_path 
	print args
	subprocess.call(args, shell=False)
	#subprocess.call(args, stdout=FNULL, stderr=FNULL, shell=False)


def scanCorn():
	global label_file_path
	if label_file_path=="":
		MessageBox=ctypes.windll.user32.MessageBoxA(0, 'Please select the label file','RoAD Warning', 0)
		return
	#proc=subprocess.Popen("C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/PlacePots.exe")###

	#FNULL = open(os.devnull, 'w')    #use this if you want to suppress output to stdout from the subprocess
	args = "C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/ScanCorn.exe " + label_file_path 
	print args
	subprocess.call(args, shell=False)

	
def takeImage():
	global label_file_path
	if label_file_path=="":
		MessageBox=ctypes.windll.user32.MessageBoxA(0, 'Please select the label file','RoAD Warning', 0)
		return
	#proc=subprocess.Popen("C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/PlacePots.exe")###

	#FNULL = open(os.devnull, 'w')    #use this if you want to suppress output to stdout from the subprocess
	args = "C:/Users/phenobot/Documents/RoAD2.0/RoAD2.0/TakeImage.exe " + label_file_path 
	print args
	subprocess.call(args, shell=False)

#GUI

label_file_path=""
corn_label_file_path=""

mGui=Tk()

mGui.title("Welcome to RoAD")

menubar=Menu(mGui)

filemenu=Menu(menubar,tearoff=0)
filemenu.add_command(label="Open",command=mOpen)
filemenu.add_command(label="Clear",command=fClear)
filemenu.add_command(label="Quit",command=mQuit)
menubar.add_cascade(label="File",menu=filemenu)

runmenu=Menu(menubar,tearoff=0)
runmenu.add_command(label="Run",command=mRun)
runmenu.add_command(label="Clear Results",command=rClear)
menubar.add_cascade(label="Run",menu=runmenu)

helpmenu=Menu(menubar,tearoff=0)
helpmenu.add_command(label="About",command=mAbout)
menubar.add_cascade(label="Help",menu=helpmenu)

save_menu=Menu(menubar,tearoff=0)
save_menu.add_command(label="Save Results to...",command=mSave)
menubar.add_cascade(label="Save",menu=save_menu)

mGui.config(menu=menubar)

image = Button(mGui, text="Take Image", command=takeImage)
image.pack(pady=10, side=BOTTOM)

corn = Button(mGui, text="Scan Corn", command=scanCorn)
corn.pack(pady=10, side=BOTTOM)

pick = Button(mGui, text="Collect Data", command=RoAdWatcher)
pick.pack(pady=10, side=BOTTOM)

place = Button(mGui, text="Experiment Setup", command=placePots)
place.pack(pady=10, side=BOTTOM)

labelFile = Button(mGui, text="Load labelFile", command=loadFile)
labelFile.pack(pady=10, side=BOTTOM)



text_input=Text(mGui,height=20,width=40)
text_input.tag_configure('big', font=('Verdana', 10, 'bold'))
text_input.pack(side=LEFT)

text_output=Text(mGui,height=20,width=40)
scroll = Scrollbar(mGui, command=text_output.yview)
text_output.configure(yscrollcommand=scroll.set)

text_output.tag_configure('big', font=('Verdana', 10, 'bold'))

text_output.pack(side=LEFT)
scroll.pack(side=RIGHT,fill=Y)


mGui.mainloop()
