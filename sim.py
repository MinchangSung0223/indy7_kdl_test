import pybullet as p
import time
import pybullet_data
import numpy as np
import math
from modern_robotics import *
from calcMR import MRSetup

def Inertia_matrix(ixx,iyy,izz,ixy,iyz,ixz):
	I = np.array([[ixx ,ixy ,ixz],[ixy ,iyy ,iyz ],[ixz ,iyz ,izz ]])
	return I

## setup
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

indy = p.loadURDF("indy7/indy7.urdf", [0, 0, -0.05],[0, 0, 0, 1])
numJoints = p.getNumJoints(indy)
p.resetBasePositionAndOrientation(indy, [0, 0, 0], [0, 0, 0, 1])
for i in range(0,numJoints):
	p.setJointMotorControl2(indy, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)


_MRSetup = MRSetup();
M = _MRSetup.getM()
Mlist = _MRSetup.getMlist()
Glist = _MRSetup.getGlist()
Slist = _MRSetup.getSlist()

g = np.array([0, 0,-9.8])
thetalist = np.array([0,0,0,0,0,0])
dthetalist = np.array([0 ,0, 0, 0 ,0 ,0]);
taulist = np.array([0,0,0,0,0,0])
Ftip = np.array([0,0,0,100,0,0])

desired_thetalist = np.array([1.57,1.57,0,0,0,0])
desired_dthetalist = np.array([0.1 ,0.1, 0.1, 0.1 ,0.1 ,0.1]);
Kp = 10;


print(Mlist.shape)
torque = 0;
while p.isConnected():
	timeStep = p.readUserDebugParameter(timeStepId)
	p.setTimeStep(timeStep)
	dt = timeStep
	jointStates = np.array(p.getJointStates(indy,[1,2,3,4,5,6]))
	q = np.array(jointStates[:,0])
	dq = np.array(jointStates[:,1])
	#G = GravityForces(np.array(jointStates), g, Mlist, Glist, Slist)
	Ftip = np.array([0,0,0,0,0,0])
	MassMatrix1 = np.array(p.calculateMassMatrix(indy,[q[0],q[1],q[2],q[3],q[4],q[5]]))
	thetalist = np.array([q[0],q[1],q[2],q[3],q[4],q[5]])
	dthetalist = np.array([dq[0],dq[1],dq[2],dq[3],dq[4],dq[5]])
	ddthetalist = np.array([0,0,0,0,0,0])
	MassMatrix2 = MassMatrix(thetalist,Mlist,Glist,Slist)
	Gmat = GravityForces(thetalist,g,Mlist,Glist,Slist);
	MRID = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
	ID= p.calculateInverseDynamics(indy,[q[0],q[1],q[2],q[3],q[4],q[5]],[dq[0],dq[1],dq[2],dq[3],dq[4],dq[5]],[0,0,0,0,0,0] )
	print((MRID-ID))
	
	torque = MRID
	#print(q)
	for j in range(1,7):
		p.setJointMotorControl2(indy, j, p.TORQUE_CONTROL, force=torque[j-1])	

	p.stepSimulation()
	time.sleep(timeStep)
