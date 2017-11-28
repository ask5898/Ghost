#!/usr/bin/env python

import pypot.dynamixel 
import time
import itertools
import numpy as np
import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String

darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
abmath = {11: 15, 12: -15, 13: -10, 14: 10, 15: -5, 16: 5}
hand = {5: 60, 6: -60}
path = "/home/ali/catkin/src/walk/scripts/data.xml"


class Dynamixel(object) :
	def __init__(self,Dxl,default_id=0) :
		ports = pypot.dynamixel.get_available_ports()
		if not ports :
			raise IOError("No ports found ")

		print "Connecting to ",ports[default_id]

		self.dxl = pypot.dynamixel.DxlIO(ports[default_id])
		self.ids = self.dxl.scan(range(25))
		print self.ids
		self.dxl.enable_torque(self.ids)
		if len(self.ids)<lock :
			raise RuntimeError("all the motors were not detected")
		self.dxl.set_moving_speed(dict(zip(self.ids,itertools.repeat(100))))


	def posWrite(self,pose) :
		pos = {ids:angle for ids,angle in pose.items()}
		self.dxl.set_goal_position(pos)
		print pos

	def listWrite(self,list) :
		pos = dict(zip(self.ids,list))
		self.dxl.set_goal_position(pos)

	def dictWrite(self,dicti) :
		
		self.dxl.set_goal_position(dicti)

	def angleWrite(self,ids,pose) :
		self.dxl.set_goal_position({ids:pose})
		
	def returnPos(self,ids) :

		return self.dxl.get_present_position((ids,))	


class XML(object) :
	def __init__(self,file) :
		try :
			tree = ET.parse(file)
			self.root = tree.getroot()
		except :
			raise RuntimeError("File not found")

	def parse(self,motion) :
		find = "PageRoot/Page[@name='" +motion+ "']/steps/step"
		try :
			steps = [x for x in self.root.findall(find)]
		except :
			raise RuntimeError("Motion not found")

		p_frame = str()
		p_pose = str()
		write = []
		for step in steps :
			write.append(Write(step.attrib['frame'],step.attrib['pose'],p_frame,p_pose))
			p_frame = step.attrib['frame']
			p_pose = step.attrib['pose']

		return write
			
	def setparse(self,motion,offset=[]) :
		find = "FlowRoot/Flow[@name='" +motion+ "']/units/unit"
		try :
			units = [x for x in self.root.findall(find)]
		except :
			raise RuntimeError("Motionset not found")
		write = []
		for unit in units :
			write.append(Writeset(unit.atrrib['main'],offset))
	
		return write


class offset(object) :
	def __init__(self,dicta,dictb) :
		self.dicta = dicta
		self.dictb = dictb			

	def setoffset(self,offset={},darwin=True) :
		if not(darwin) :
			pass

		else :
			for key in offset.keys() :
				if offset[key] == 'i' :
					self.dicta[key] = -self.dicta[key]
					self.dictb[key] = -self.dictb[key]
				else :
					self.dicta[key] += offset[key]
					self.dictb[key] += offset[key]		


	def applyoffset(self,offsets) :
		map(lambda offset:self.setoffset(offset),offsets)
		return self.dicta,self.dictb

	
class Write(object) :
	def __init__(self,frame,pose,p_frame,p_pose) :
		self.frame = int(frame)
		self.begin = {}
		self.end = {}

		if not(p_pose) :
			self.frame_diff = 1
			p_pose = pose
		else :
			self.frame_diff = self.frame-int(p_frame) 

		for ids,pos in enumerate(map(float,p_pose.split())) :
			self.end[ids+1] = pos	

		for ids,pos in enumerate(map(float,pose.split())) :
			self.begin[ids+1] = pos

		
	def setoffset(self,offset=[]) :
		off = offset(self.end,self.begin)
		self.end,self.begin = off.applyoffset(offset)


	def motion(self,speed=1) :
		print self.begin
		print self.end
		write = []
		ids = []
		for key in self.end.keys() :
			linp = np.linspace(self.end[key],self.begin[key],self.frame_diff)
			write.append(linp)
			ids.append(key)	

		print "out"
		for pose in zip(*write) :
			print "in"
			dxl.posWrite(dict(zip(ids,pose)))
			time.sleep(0.008/speed)



class Writeset(object) :
	def __init__(self,main,write,speed=1,offset=[],motion=[]) :
		self.motion = motion
		self.offset = offset
		self.motion.append(main)
		self.speed = speed
		self.eternity = True

	def run(self,count=None,speed=1) :

		if count is not None :
			self.eternity = False
		speed = self.speed
		while count>0 :
			for motion in self.motion :
				for offset in self.offset :
					motion.applyoffset(offset)
					motion.motion(speed)
				
			if self.eternity :
				continue
		
			count -= 1 
			
				
				

class custommotion(object) :
	def __init__(self,motionset) :
		self.motionset = motionset

	def run(self) :
		#prev_motionset = str()
		for motionset in self.motionset :
			speed = motionset.speed
			motionset.run(speed)

		

xml = XML(path)
w1 = Writeset(xml.parse(motion="32 F_S_L"),speed=2.1,offset=[darwin])
w2 = Writeset(xml.parse(motion="33 "),speed=2.1,offset=[darwin])
w3 = Writeset(xml.parse(motion="38 F_M_R"),speed=2.7,offset=[darwin])
w4 = Writeset(xml.parse(motion="39 "),speed=2.1,offset=[darwin])
w5 = Writeset(xml.parse(motion="36 F_M_L"),speed=2.7,offset=[darwin])
w6 = Writeset(xml.parse(motion="37 "),speed=2.1,offset=[darwin])
walk_init = custommotion(motionset=[w1,w2])
walk_motion = custommotion(motionset=[w3,w4,w5,w6])				

balance = Writeset(xml.parse(motion="152 Balance"),offset=[darwin,hand])

				 
if __name__ == "__main__" :
	dxl = Dynamixel(Dxl=20)
	balance.run()	
	raw_input("Proceed?")		
	walk_init.run()
	while True :
		walk_motion.run()
	





