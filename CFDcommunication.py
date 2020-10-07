# -*- coding: utf-8 -*-
"""
Created on Thu Oct  17 17:28:03 2019

@author: andrea
"""

import numpy as np
import time
import collections

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class CFDcommunication:

    def __init__(self,config):
        
        self.config = config
       
        # initial and target conditions from config file
        self.xA = config["XA"]
        self.yA = config["YA"]
        self.A = np.array([self.xA,self.yA])
        self.uA = config["UA"]
        self.vA = config["VA"]
        self.xB = config["XB"]
        self.yB = config["YB"] 
        self.B = np.array([self.xB,self.yB])
        self.rhoAB = np.linalg.norm(self.A-self.B)
        self.diffyAB_init = abs(self.yA-self.yB)

        self.BA = self.A-self.B
        self.phiA = np.arctan2(self.BA[1],self.BA[0])
        
        # state initialisation
        self.cartesian_init = np.array([self.xA,self.yA, self.uA, self.vA])
        self.state = self.get_state_in_relative_polar_coordinates(self.cartesian_init)
        
        # attributs needed by the sureli code or gym wrappers
        self.action_space = collections.namedtuple('action_space', ['low', 'high', 'shape'])(-100, 100, (1,))
        self.action_size = 1
        self.observation_space = collections.namedtuple('observation_space', ['shape'])(self.cartesian_init.shape)
        self.reward_range = None
        self.metadata = None

        # B coords array (size depending on update number)
        self.nb_ep = 0
        self.nb_pointB_change = 0
        self.B_array = np.zeros([self.config["MAX_EPISODES"]//self.config["POINTB_CHANGE"]+1, 2])
        self.B_array[self.nb_pointB_change, :] = self.B

        # dt_array and var_array initialisation (for plotting purposes only, not required by the application)    
        self.var_episode = [0]
        self.variables = ['x', 'y', 'u', 'v', 'actions', 'rewards']
        self.dt_array = np.array([i*config["DELTA_TIME"] for i in range(config["MAX_STEPS"]+1)])
        self.var_array = np.zeros([len(self.variables), config["MAX_EPISODES"],config["MAX_STEPS"]+1])
        # fill array with initial state
        for i in range(len(self.cartesian_init)):
            self.var_array[i,:,0] = self.cartesian_init[i]
        
        #set of routes and files that permit access to STAR CCM+ and Python
        self.startstoproute = 'cfd/starccm/startstop/'
        self.exporteddataroute = 'cfd/starccm/exporteddata/'
        
        self.finishsimulation = 'finishsimulation.txt'
        self.finishsimulationflag = 'finishsimulationflag.txt'
        
        self.pitchflag = 'pitchflag.txt'
        self.actiontoCFD = 'actiontoCFD.txt'
        
        self.resetsimulation = 'resetsimulation.txt'
        self.resetsimulationflag = 'resetsimulationflag.txt'
        
        self.stepdone = 'stepdone.txt'
        self.stepdoneflag = 'stepdoneflag.txt'

        self.filetranslationx = 'translationx.txt'
        self.filetranslationy = 'translationy.txt'
        self.filevelocityx = 'velocityx.txt'
        self.filevelocityy = 'velocityy.txt'
        self.fileaccelerationx = 'accelerationx.txt'
        self.fileaccelerationy = 'accelerationy.txt'

	#initialise the files for the simulation
        self.clearfiles()
        self.initialisefiles()
        

    def step(self,action):
        # to compute the step:
        # 1- export the action to external file to be read by STARCCM+
        # 2- wait until iteration is performed in STARCCM+ and information written
        # to text file. To do so, continuosly check file "endinteration"
        # 3- read text file with information of the state computed ini STARCCM+
        # 4- compute reward and done

        old_polar_state = self.state        
        self.pitchrate = action
        action = action + np.random.normal(scale=self.config["ACTION_SIGMA"])
        
        #write the action computed by DDPG to STARCCM+
        self.sendactionCFD(action)
        
        fileroute = self.startstoproute
        
        filenameflag = self.stepdoneflag
        lookforflag = '0.0'
        writeflag = '1.0\n'
        
        filename = self.stepdone
        lookforvalue = '1'
        writevalue = '0'
        
        checkflag_writedataTXT(fileroute,filenameflag,lookforflag,writeflag, fileroute,filename,lookforvalue,writevalue)
        
        self.waitSTARCCM()
        
        new_cartesian_state = self.readstatesCFD()
        new_polar_state = self.get_state_in_relative_polar_coordinates(new_cartesian_state)
        
        self.state = new_polar_state
        
        #compute reward and done
        reward = self.compute_reward(old_polar_state, action, new_polar_state)
        won, lost = self.is_won_or_lost(new_polar_state)
        done = self.isdone(won,lost)
        if done:
            reward = self.update_reward_if_done(reward, won, lost)
        
        # save data for printing 
        self.var_episode.append([new_cartesian_state[0], new_cartesian_state[1], new_cartesian_state[2], new_cartesian_state[3], self.pitchrate, reward])

        return [self.state, reward, done, None]


    def reset(self, state=None):
        self.nb_ep +=1
        self.var_episode = []
        if np.mod(self.nb_ep,self.config["POINTB_CHANGE"]) == 0:
            self.update_B()

        if state==None:
            self.state = self.get_state_in_relative_polar_coordinates(self.cartesian_init)
        else:
            self.state = state

        self.clearfiles()
        fileroute = self.startstoproute
        fileresetflag = self.resetsimulationflag
        lookforflag = ''
        writeflag = '1.0\n'
        
        filereset = self.resetsimulation
        lookforvalue = ''
        writevalue = '1'
        
        checkflag_writedataTXT(fileroute,fileresetflag,lookforflag,writeflag, fileroute,filereset,lookforvalue,writevalue)
        
        return self.state


    # defined in order to mimic a gym environment
    def render(self, mode='human'):
        pass


    # defined in order to mimic a gym environment
    def close(self):
        pass


    # defined in order to mimic a gym environment
    def seed(self):
        pass


    def compute_reward(self, old_polar_state, action, new_polar_state):
        delta_rho = (new_polar_state[0] - old_polar_state[0])/self.rhoAB
        delta_abs_theta = np.abs(new_polar_state[1]) - np.abs(old_polar_state[1])
        
        reward = -10000*np.abs(new_polar_state[1])/(np.pi/2)
       
        return reward


    def update_reward_if_done(self, reward, won, lost):
        if won: reward += 10000
        elif lost: reward += -10000

        return reward


    def isdone(self, won, lost):
        done = False
        if won or lost:
            done = True

        return done


    def is_won_or_lost(self, polar_state):
        won = False
        lost = False

        if np.abs(polar_state[0]/self.rhoAB) <= 10**(-3):
            won = True
        elif np.abs(polar_state[1]+self.phiA) >= np.pi/2.:
            lost = True

        return won, lost


    def print_won_or_lost(self, polar_state):
        won = False
        lost = False

        if np.abs(polar_state[0]/self.rhoAB) <= 10**(-3):
            won = True
            print('won')
        elif np.abs(polar_state[1]+self.phiA) >= np.pi/2.:
            lost = True
            print('lost')

        return won, lost


    # update B coordinates as required in the CFD config file
    def update_B(self):
        self.nb_pointB_change +=1

        self.xB = np.random.uniform(self.xA, self.config["XB"])
        self.yB = np.random.uniform(self.yA-self.diffyAB_init, self.yA+self.diffyAB_init)

        # keep iterating until the absolute angle between A and B is below 10 degrees
        while abs(np.arctan2(abs(self.yB-self.yA),abs(self.xB-self.xA))) > self.threshold_angle*np.pi/180:
            self.xB = np.random.uniform(self.xA, self.config["XB"])
            self.yB = np.random.uniform(self.yA-self.diffyAB_init, self.yA+self.diffyAB_init)

        print('Final absolute angle',abs(np.arctan2(abs(self.yB-self.yA),abs(self.xB-self.xA)))/np.pi*180, 'degrees')
        print('Final point coordinates: (',self.xB,self.yB,')')

        self.B = np.array([self.xB,self.yB])
        self.rhoAB = np.linalg.norm(self.A-self.B)
        self.BA = self.A-self.B
        self.phiA = np.arctan2(self.BA[1],self.BA[0])

        self.B_array[self.nb_pointB_change, :] = self.B


# Below are only utility functions ------------------------------------------

    def get_state_in_relative_polar_coordinates(self, cartesian_state):
        BP = cartesian_state[0:2]-self.B
        rho = np.linalg.norm(BP)
        phiP = np.arctan2(BP[1],BP[0])
        theta = phiP - self.phiA

        u = cartesian_state[2]
        v = cartesian_state[3]

        rhoDot = u * np.cos(phiP) + v * np.sin(phiP)
        thetaDot = - u * np.sin(phiP) + v * np.cos(phiP)
        polar_state = np.array([rho, theta, rhoDot, thetaDot])

        return polar_state


    def readstatesCFD(self):
        
        translationx = readTXT(self.exporteddataroute,self.filetranslationx)
        translationy = readTXT(self.exporteddataroute,self.filetranslationy)
        velocityx = readTXT(self.exporteddataroute,self.filevelocityx)
        velocityy = readTXT(self.exporteddataroute,self.filevelocityy)        

        newstatecartesian = [translationx, translationy, velocityx, velocityy]
        return newstatecartesian


    def finishCFD(self,done=False):
        #done: boolean that takes the values
        # True -> finish the simulation
        # False -> not finish the simulation, just update the value of the flag
        fileroute = self.startstoproute
        filefinish = self.finishsimulation
        filefinishflag = self.finishsimulationflag
        filestepdoneflag = self.stepdoneflag
        
        flag = ''
        while '0.0' not in flag:
            with open(fileroute+filefinishflag,'r+') as f:
                flag = f.read()
                time.sleep(0.05)
            f.close()

        #in fact, not needed condition since it would not have passed the while loop    
        if '0.0' in flag:
            #update the flag to permit continue to STAR CCM+
            flag = '1.0\n'
            if done==True:
                with open(fileroute+filefinish,'r+') as f2:
                    f2.seek(0)
                    f2.write('1')
                    f2.truncate()      
#                   #update flag for finishsimulationflag
#                   flag = '1.0\n'
                    with open(fileroute+filefinishflag,'r+') as f:
                        f.seek(0)
                        f.write(flag)
                        f.truncate()
                    with open(fileroute+filestepdoneflag,'r+') as f3:
                        f3.seek(0)
                        f3.write(flag)
                        f3.truncate()
                    f3.close()
                f2.close()
            else:
#                #update the flag to permit continue to STAR CCM+
#                flag = '1.0\n'
                with open(fileroute+filefinishflag,'r+') as f:
                    f.seek(0)
                    f.write(flag)
                    f.truncate()
                f.close()
            
            
    def clearfiles(self):
        
        clearTXT(self.exporteddataroute,self.actiontoCFD)
        clearTXT(self.exporteddataroute,self.filetranslationx)
        clearTXT(self.exporteddataroute,self.filetranslationy)
        clearTXT(self.exporteddataroute,self.filevelocityx)
        clearTXT(self.exporteddataroute,self.filevelocityy)
        clearTXT(self.exporteddataroute,self.fileaccelerationx)
        clearTXT(self.exporteddataroute,self.fileaccelerationy)


    def initialisefiles(self):
        #check and set as not step done in STARCCM+ simulation
        datastr = '1'
        writeTXT(self.startstoproute,self.stepdone,datastr)
        
        datastr = '0.0\n'
        writeTXT(self.startstoproute,self.stepdoneflag,datastr)
                
        #check and set as not to finish the STARCCM+ simulation
        datastr = '0'
        writeTXT(self.startstoproute,self.finishsimulation,datastr)
        
        datastr = '0.0\n'
        writeTXT(self.startstoproute,self.finishsimulationflag,datastr)

        #check and set as reset for the STARCCM+ simulation
        datastr = '0.0\n'
        writeTXT(self.startstoproute,self.resetsimulation,datastr)

        datastr = '0.0\n'
        writeTXT(self.startstoproute,self.resetsimulationflag,datastr)
        
        #check no pitch rate has been computed
        datastr = '0.0\n'
        writeTXT(self.startstoproute,self.pitchflag,datastr)
        

    # export the action information to external .csv file to be read by STARCCM
    def sendactionCFD(self,action):
        routeflag = self.startstoproute
        fileflag = self.pitchflag
        lookforflag = '0.0'
        writeflag = '1.0\n'
        
        routeexchangefiles = self.exporteddataroute
        actionfile = self.actiontoCFD
        #not looked for value, so it is left 'empty'
        lookforvalue = ''
        writevalue = str(action[0])
        
        checkflag_writedataTXT(routeflag,fileflag,lookforflag,writeflag, routeexchangefiles,actionfile,lookforvalue,writevalue)
    
    
    # wait until complete iteration of STARCCM+
    # continuously read a file to check if it has written on it '1', when it has written '1'
    #it means that the step in STARCCM+ is over
    def waitSTARCCM(self):
    
        fileroute = self.startstoproute
        filenameflag = self.stepdoneflag
        fileresetflag = self.resetsimulationflag
    
        #check that the turn of python has arrived to continue with the evaluation
        flag = ''
        while '0.0' not in flag:
            with open(fileroute+filenameflag,'r+') as f:
                flag = f.read()
                time.sleep(0.05)
            f.close()
    
        #change the flag of the reset file too, not only the one of the step to keep
        #on with the shift of DDPG and STAR CCM+ evaluation
        resetflag = ''
        while '0.0' not in resetflag:
            with open(fileroute+fileresetflag,'r+') as f:
                resetflag = f.read()
                #print(flag + "\n")
                time.sleep(0.05)
            f.close()
        #in fact, not needed condition since it would not have passed the while loop
        if '0.0' in resetflag:
            with open(fileroute + fileresetflag, 'r+') as f:
                f.seek(0)
                f.write('1.0\n')
                f.truncate()
            f.close()


    def fill_array_tobesaved(self):
        for i in range(len(self.variables)):
            for k in range(len(self.var_episode)):
                self.var_array[i, self.nb_ep-1, k+1] = self.var_episode[k][i]


    def print_array_in_files(self, folder):
        for i, var in enumerate(self.variables):
            filename = folder+'/'+var+'.csv'
            np.savetxt(filename, self.var_array[i,:,:], delimiter=";")

        filename = folder+'/Bcoordinates.csv'
        np.savetxt(filename, self.B_array, delimiter=";")

        filename = folder+'/time.csv'
        np.savetxt(filename, self.dt_array, delimiter=";")


    def plot_training_output(self, rewards, folder):
        # TODO optimize this to avoid recomputing everything
        xfirst = np.trim_zeros(self.var_array[0,0,:], 'b')
        yfirst = self.var_array[1,0,:len(xfirst)]
        xlast = np.trim_zeros(self.var_array[0,-1,:], 'b')
        ylast = self.var_array[1,-1,:len(xlast)] 

        cumulative_reward = self.var_array[5,:,:].sum(axis=1)

        best = np.argmax(cumulative_reward)
        xbest = np.trim_zeros(self.var_array[0,best,:], 'b')
        ybest = self.var_array[1,best,:len(xbest)]

        worst = np.argmin(cumulative_reward)
        xworst = np.trim_zeros(self.var_array[0,worst,:], 'b')
        yworst = self.var_array[1,worst,:len(xworst)]

        plt.cla()
        plt.figure(figsize=(10, 5))
        plt.tight_layout()
        plt.suptitle(folder.rsplit('/', 1)[1])
        plt.subplot(1,2,1)
        plt.title('Trajectories')
        plt.plot([self.xA,self.xB], [self.yA,self.yB], color='black', ls='--', label='Ideal path')
        plt.plot(xfirst, yfirst, label='first path')
        plt.plot(xlast, ylast, label='last path ep='+str(self.config['MAX_EPISODES']))
        plt.plot(xbest, ybest, label='best path ep='+str(best))
        plt.plot(xworst, yworst, label='worst path ep='+str(worst))
        plt.grid()
        plt.xlabel('x (m)', fontsize=14)
        plt.ylabel('y (m)', fontsize=14)
        plt.legend(fontsize = 10, loc='best')

        plt.subplot(1,2,2)
        plt.title('Training reward')
        plt.plot(rewards, color='k')
        plt.grid()
        plt.xlabel('Episode', fontsize=14)
        plt.ylabel('Reward', fontsize=14)
        plt.savefig(f'{folder}/train_output.png')

    def plot_testing_output(self, rewards, folder):
        # TODO optimize this to avoid recomputing everything
        score = sum(rewards)/len(rewards) if rewards else 0
        xlast = np.trim_zeros(self.var_array[0,-1,:], 'b')
        ylast = self.var_array[1,-1,:len(xlast)]

        filex_train = f'{folder}/../x.csv'
        filey_train = f'{folder}/../y.csv'
        filereward_train = f'{folder}/../rewards.csv'
        xmatrix_train = np.loadtxt(filex_train, delimiter=";")
        ymatrix_train = np.loadtxt(filey_train, delimiter=";")
        rewardmatrix_train = np.loadtxt(filereward_train, delimiter=";")
        xlasttrain = np.trim_zeros(xmatrix_train[-1,:], 'b')
        ylasttrain = ymatrix_train[-1,:len(xlasttrain)]

        cumulative_reward = rewardmatrix_train.sum(axis=1)

        plt.cla()
        plt.figure(figsize=(10, 5))
        plt.suptitle(folder.rsplit('/', 2)[1])
        plt.subplot(1,2,1)
        plt.plot([self.xA,self.xB], [self.yA,self.yB], color='black', ls='--', label='Ideal path')
        plt.plot(xlasttrain, ylasttrain, label='last training path')
        plt.plot(xlast, ylast, label='test path')
        plt.grid()
        plt.xlabel('x (m)', fontsize=14)
        plt.ylabel('y (m)', fontsize=14)
        plt.legend(fontsize = 10, loc='best')

        t='Rewards \n'+\
          f"   Test         = {score:9.0f}\n"+\
          f"   Train (last) = {cumulative_reward[-1]:9.0f}"

        left, width = .25, .5
        bottom, height = .25, .5
        right = left + width
        top = bottom + height

        ax = plt.subplot(1,2,2)
        ax.axis('off')
        ax.text(0.5*(left+right), 0.5*(bottom+top), t,
                horizontalalignment='center',
                verticalalignment='center',
                transform=ax.transAxes,
                color='k',
                fontsize=12)

        plt.savefig(f'{folder}/test_output.png')

    
def readTXT(fileroute,filename):
    readdata=[]
    try:
        with open(fileroute+filename,'r+') as f:
            readdata = f.read()
        data = float(readdata[:-1])
    finally:
        f.close()
    return data


def writeTXT(fileroute,filename,datastr):
    try:
        with open(fileroute+filename,'r+') as f:
            f.seek(0)
            f.write(datastr)
            f.truncate()
    finally:
        f.close()
        
        
def clearTXT(fileroute,filename):
    try:
        with open(fileroute+filename,'r+') as f:
            f.seek(0)
            f.truncate()
    finally:
        f.close()        
        
#function that checks the semaphore to write the new values on the corresponding files
def checkflag_writedataTXT(routeflag,fileflag,lookforflag,writeflag, routefile,file,lookforvalue,writevalue):
    #wait until the condition of the flag that is given as input is satisfied
    #for this problem, the two values of the flag are:
    #0 -> only Python can change the file
    #1 -> only Java can change the file
    flag = ''
    while lookforflag not in flag:
        with open(routeflag+fileflag,'r+') as f:
            flag=f.read()
            #delay to avoid failure in files because of open and close at the 'same time'
            time.sleep(0.05)
        f.close()
        
    #in fact, not needed condition since it would not have passed the while loop
    if lookforflag in flag:
        with open(routefile+file,'r+') as f2:
            readvalue = f2.read()
            #check that the condition that is wanted in the file is fulfilled and
            #update the value according to the requirements
            if lookforvalue in readvalue:
                value = writevalue
                f2.seek(0)
                f2.write(value)
                f2.truncate()
                #change the value of the flag so that STAR CCM+ can access to the file
                with open(routeflag+fileflag,'r+') as f:
                    f.seek(0)
                    f.write(writeflag)
                    f.truncate()
                f.close()
        f2.close()
