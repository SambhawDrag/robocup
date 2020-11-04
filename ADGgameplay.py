import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import Ref
from role import _GoToPoint_, GoToPoint, DribbleKick, defenders
from tactics import Goalie
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import *
from math import atan2,pi
from utils.functions import *
import multiprocessing
import threading
import time

#pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
#command = 5
flag = 0

def attackfunc(id_1,state,pub,target):
	kub = kubs.kubs(id_1,state,pub)
	kub.update_state(state)
	print("Attacker called !!!")
	print('BOT_ID: ',id_1)
	print(kub.kubs_id)
	#---Can be made dynamic as per the Goalie's position
	dribblekick_fsm = DribbleKick.DribbleKick(target)
	dribblekick_fsm.add_kub(kub)

	dribblekick_fsm.spin()
	kub.reset()
	kub.execute()

def defendfunc_up(id_1,id_2,state,pub):
	kub = kubs.kubs(id_1,state,pub)
	kub.update_state(state)
	print("Defender 1 called !!!")
	print('BOT_ID: ',id_1)

	print(kub.kubs_id)
	defend_fsm = defenders.defender(id_1,id_2,"top")
	defend_fsm.add_kub(kub)
	print('something before spin')
	defend_fsm.spin()

def defendfunc_down(id_1,id_2,state,pub):
	kub = kubs.kubs(id_2,state,pub)
	kub.update_state(state)
	print("Defender 2 called !!!")
	print('BOT_ID: ',id_1)

	print(kub.kubs_id)
	defend_fsm = defenders.defender(id_2,id_1,"bottom")
	defend_fsm.add_kub(kub)
	
	print('something before spin')
	defend_fsm.spin()

def goaliefunc(id_1,state,pub):
	kub = kubs.kubs(id_1,state,pub)
	
	kub.update_state(state)
	print("Goalie called !!!")
	print('BOT_ID: ',id_1)

	g_fsm = Goalie.Goalie()
	g_fsm.add_kub(kub)

	g_fsm.as_graphviz()
	g_fsm.write_diagram_png()
	print('something before spin')
	g_fsm.spin()


# ----- ATTACKER
def main1(process_id, pub):
	# global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass

	while True:
		state = None
		rospy.wait_for_service('bsServer',)
		getState = rospy.ServiceProxy('bsServer',bsServer)
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print("chutiya")		
		if state:
			kub = kubs.kubs(process_id,state,pub)
			kub.reset()
			kub.execute()
			print("process 1: ATTACKER :",command)
			
			target = Vector2D(4500,0)
			attackfunc(1,state.stateB,pub,target)
			kub.reset()
			kub.execute()
			print(process_id)
			print("COMPLETED-attackonce")
			time.sleep(1)

#---DEFENDER-UP		
def main2(process_id, pub):
	# global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass	

	while True:
		state = None
		# state=shared.get('state')
		rospy.wait_for_service('bsServer',)
		getState = rospy.ServiceProxy('bsServer',bsServer)
		try:
				state = getState(state)
		except rospy.ServiceException, e:
				print("chutiya")		
		if state:
			kub = kubs.kubs(process_id,state,pub)
			kub.reset()
			kub.execute()
			print("process 2 ",command)
				
			defendfunc_up(2,3,state.stateB,pub)
			kub.reset()
			kub.execute()
			print(process_id)
			print("COMPLETED")
			time.sleep(1)
			break	

#--- DEFENDER-DOWN
def main3(process_id, pub):
	# global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass	

	while True:
		state = None
		# state=shared.get('state')
		rospy.wait_for_service('bsServer',)
		getState = rospy.ServiceProxy('bsServer',bsServer)
		try:
				state = getState(state)
		except rospy.ServiceException, e:
				print("chutiya")		
		if state:
			kub = kubs.kubs(process_id,state,pub)
			kub.reset()
			kub.execute()
			print("process 2 ",command)
				
			defendfunc_down(2,3,state.stateB,pub)
			kub.reset()
			kub.execute()
			print(process_id)
			print("COMPLETED")
			time.sleep(1)
			break	

#--- GOALIE
def main4(process_id, pub):
	#pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try:
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass

	while True:
		state = None
		# state=shared.get('state')
		rospy.wait_for_service('bsServer',)
		getState = rospy.ServiceProxy('bsServer',bsServer)
		try:
				state = getState(state)
		except rospy.ServiceException, e:
				print("Error ", e)		
		if state:
				#print('lasknfcjscnajnstate',state.stateB.homePos)
				#p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
				print("kyun")
				goaliefunc(0,state.stateB,pub)

#---Execution:
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

# Bot0-Goalie | Bot1-attacker | Bot2, Bot3 - Defenders
p1 = multiprocessing.Process(target=main1, args=(1,pub))
p2 = multiprocessing.Process(target=main2, args=(2,pub))
p3 = multiprocessing.Process(target=main3, args=(3,pub))
p4 = multiprocessing.Process(target=main4, args=(4,pub))

p1.start()
p2.start()
p3.start()
p4.start()
p1.join()
p2.join()
p3.join()
p4.join()

print("POSITIONS TAKEN-----")		