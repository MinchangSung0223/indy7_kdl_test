import kdl_parser_py.urdf
import PyKDL as kdl
import numpy as np
filename = "./indy7/indy7.urdf"
(ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
print("getNrOfJoints : ", tree.getNrOfJoints())
print("getNrOfSegments : ", tree.getNrOfSegments())
chain = tree.getChain("base_link", "tcp")
print("NrOfJoints : ", chain.getNrOfJoints())
num_joints = chain.getNrOfJoints()
q = kdl.JntArray(num_joints)
q_dot = kdl.JntArray(num_joints)
q[0] = 0
q[1] = 0
q[2] = 0
q[3] = 0
q[4] = 0
q[5] = 0
q_dot[0] = 0
q_dot[1] = 0
q_dot[2] = 0
q_dot[3] = 0
q_dot[4] = 0
q_dot[5] = 0

fk_ee = kdl.ChainFkSolverPos_recursive(chain)
jac_ee = kdl.ChainJntToJacSolver(chain)
T = kdl.Frame()
fk_ee.JntToCart(q, T)
print("=========================T===========================")
print(T)

Jacobian = kdl.Jacobian(num_joints)
ret = jac_ee.JntToJac(q,Jacobian)
print("=====================Jacobian=======================")
print(Jacobian)


grav_vector = kdl.Vector(0,0,-9.8)
MassMatrix = kdl.JntSpaceInertiaMatrix(num_joints)
Gravity = kdl.JntArray(num_joints)
Coriolis = kdl.JntArray(num_joints)
dyn_ee = kdl.ChainDynParam(chain,grav_vector)
#JntToMass 
ret = dyn_ee.JntToCoriolis(q,q_dot,Coriolis)
ret = dyn_ee.JntToMass(q,MassMatrix)
ret = dyn_ee.JntToGravity(q,Gravity)
print("=====================MassMatrix=======================")
print(MassMatrix)
print("======================Gravity=========================")
print(Gravity)
print("======================Coriolis========================")
print(Coriolis)
