
import modern_robotics as mr
import numpy as np
import scipy.io as sio
mat_contents = sio.loadmat("indy7_params.mat")

def Inertia_matrix(ixx,ixy,ixz,iyy,iyz,izz):
    I = np.array([[ixx ,ixy ,ixz],[ixy ,iyy ,iyz ],[ixz ,iyz ,izz ]])
    return I
class MRSetup():
    def __init__(self):
        H1 = 0.3
        H2 = 0.45
        H3 = 0.350
        H4 = 0.228
        W1 = 0.0035
        W2 = 0.183

        S1 = np.array([0, 0,  1,  0, 0,  0])
        S2 = np.array([0, -1,  0, H1,0 ,  0])
        S3 = np.array([0, -1,  0, H1+H2, 0, 0])
        S4 = np.array([0, 0,   1, -W1, 0, 0])
        S5 = np.array([0, -1,  0, H1+H2+H3, 0, 0])
        S6 = np.array([0, 0,  1, -W1-W2,0, 0])

        M = np.array([[1 ,0 ,0 ,0],
            [0 ,1 ,0 ,-W1-W2 ],
            [0, 0 ,1 ,H1+H2+H3+H4],
            [0, 0, 0 ,1 ]]);

        M01 = np.array([[1 ,0, 0, 0],
               [0 ,1 ,0 ,0],
               [0 ,0 ,1 ,H1],
               [0 ,0 ,0 ,1]])
        M12 = np.array([[0 , -1  , 0  , 0],
              [0 ,  0   ,-1   ,0],
              [1  , 0  ,  0  , 0],
              [0  , 0 ,   0  , 1]])
        M23 = np.array([[0 ,-1 ,0, H2],
               [1 ,0, 0 ,0],
               [0 ,0, 1, 0],
               [0 ,0, 0 ,1]])
        M34 = np.array([[-1, 0 , 0, 0],
               [0 ,0 , -1 ,-H3],
               [0 ,-1, 0 ,W1],
               [0 ,0 ,0 ,1 ]])
        M45 = np.array([[1 ,0, 0, 0],
              [0  ,0 ,-1, -W2 ],
              [0  ,1 ,0 ,0],
              [0  ,0 ,0 ,1 ]])
        M56 = np.array([[1, 0, 0,0],
              [0 ,0 ,1 ,H4],
              [0 ,-1 ,0 ,0],
              [0 ,0 ,0 ,1 ]])

        #I0 = Inertia_matrix(0.00572623,0.00558989,0.00966674,0.00000251,-0.0000014,-0.00011380);
        #I1 = Inertia_matrix(0.15418559,0.12937017,0.05964415,-0.00000235,-0.04854267,0.00001739);
        #I2 = Inertia_matrix(0.2935698,0.28094142,0.03620609,-0.0000004,0.03727972,0.00001441);
        #I3 = Inertia_matrix(0.03424593,0.03406024,0.00450477,0.00000149,0.00186009,0.00000724);
        #I4 = Inertia_matrix(0.00670405,0.00279246,0.00619341,0.00000375,-0.00127967,0.0000015);
        #I5 = Inertia_matrix(0.00994891,0.00978189,0.00271492,0.00000014,-0.00093546,0.00000321);
        #I6 = Inertia_matrix(0.00043534,0.00044549,0.00059634,0.00000013,0.00000051,-0.00000002);

        I0= Inertia_matrix(+0.00572623,+0.00000251,-0.00011380,+0.00558959,-0.00000014,+0.00966674)
        I1= Inertia_matrix(+0.15418559,-0.00000235,+0.00001739,+0.12937017,-0.04854267,+0.05964415)
        I2= Inertia_matrix(+0.29356980,-0.00000040,+0.00001441,+0.28094142,+0.03727972,+0.03620609)
        I3= Inertia_matrix(+0.03424593,+0.00000149,+0.00000724,+0.03406024,+0.00186009,+0.00450477)
        I4= Inertia_matrix(+0.00670405,+0.00000375,+0.00000150,+0.00279246,-0.00127967,+0.00619341)
        I5= Inertia_matrix(+0.00994891,+0.00000014,+0.00000321,+0.00978189,-0.00093546,+0.00271492)
        I6= Inertia_matrix(0.00043534,+0.00000013,-0.00000002,0.00044549,+0.00000051,0.00059634)



        mass0 = 1.59306955;
        mass1 = 11.8030102;
        mass2 = 7.99292141;
        mass3 = 2.99134127;
        mass4 = 2.12317035;
        mass5 = 2.28865091;
        mass6 = 0.40083918;

        G0 = np.eye(6)*mass0;
        G0[0:3,0:3] = I0;
        G1 = np.eye(6)*mass1;
        G1[0:3,0:3] = I1;
        G2 = np.eye(6)*mass2;
        G2[0:3,0:3] = I2;
        G3 = np.eye(6)*mass3;
        G3[0:3,0:3] = I3;
        G4 = np.eye(6)*mass4;
        G4[0:3,0:3] = I4;
        G5 = np.eye(6)*mass5;
        G5[0:3,0:3] = I5;
        G6 = np.eye(6)*mass6;
        G6[0:3,0:3] = I6;
        self.Glist = np.array([G1, G2, G3,G4,G5,G6])
        self.M = M;
        Mlist = np.array(mat_contents["Mlist"])
        Glist = np.array(mat_contents["Glist"])
        self.Mlist = np.array([Mlist[:,:,0],Mlist[:,:,1],Mlist[:,:,2],Mlist[:,:,3],Mlist[:,:,4],Mlist[:,:,5],Mlist[:,:,6]])
        self.Glist = np.array([Glist[:,:,0],Glist[:,:,1],Glist[:,:,2],Glist[:,:,3],Glist[:,:,4],Glist[:,:,5]])
        #self.Mlist = np.array([np.eye(4),M01,M12,M23,M34,M45,M56,M])
        self.Slist = np.array([S1,S2,S3,S4,S5,S6]).T


    def getM(self):
        return self.M
    def getSlist(self):
        return self.Slist
    def getMlist(self):
        return self.Mlist
    def getGlist(self):
        return self.Glist

