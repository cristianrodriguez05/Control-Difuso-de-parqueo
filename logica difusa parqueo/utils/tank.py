import sys
import math
from api import vrep
import numpy as np
import matplotlib.pyplot as plt


class Distances:

    def __init__(self, en, en2, es, es2, ne, ne2, nw, nw2, se, se2, sw, sw2, wn, wn2, ws, ws2):
        self.en = min(en, 6)
        self.es = min(es, 6)
        self.ne = min(ne, 6)
        self.nw = min(nw, 6)
        self.se = min(se, 6)
        self.sw = min(sw, 6)
        self.wn = min(wn, 6)
        self.ws = min(ws, 6)

        self.en2 = min(en2, 6)
        self.es2 = min(es2, 6)
        self.ne2 = min(ne2, 6)
        self.nw2 = min(nw2, 6)
        self.se2 = min(se2, 6)
        self.sw2 = min(sw2, 6)
        self.wn2 = min(wn2, 6)
        self.ws2 = min(ws2, 6)

    def __repr__(self):
        return f'NW:{self.nw:.2f} NE:{self.ne:.2f} WN:{self.wn:.2f} EN:{self.en:.2f} | SW:{self.sw:.2f} SE:{self.se:.2f} WS:{self.ws:.2f} ES:{self.es:.2f}'


class Tank:
    def __init__(self):
        self.clientID = self.connect()
        #obtener identificadores para los controladores de robots
        err_code, self.left_front_handle =  vrep.simxGetObjectHandle(self.clientID,'left_front', vrep.simx_opmode_blocking)
        err_code, self.left_back_handle  =  vrep.simxGetObjectHandle(self.clientID,'left_back', vrep.simx_opmode_blocking)
        err_code, self.right_back_handle =  vrep.simxGetObjectHandle(self.clientID,'right_back', vrep.simx_opmode_blocking)
        err_code, self.right_front_handle=  vrep.simxGetObjectHandle(self.clientID,'right_front', vrep.simx_opmode_blocking)
        
        self.side_handles=[]
        for l in 'rl':
            for i in range(1,7):
                err_code, handle=  vrep.simxGetObjectHandle(self.clientID,'sj_'+l+'_'+str(i) , vrep.simx_opmode_blocking)
                self.side_handles.append(handle)
       
        #velocidad inicial
        self.leftvelocity=0
        self.rightvelocity=0
        self.MaxVel=10
        self.dVel=1

        #proximidad
        self.proximity_sensors = ["EN", "ES", "NE", "NW", "SE", "SW", "WN", "WS"]
        self.proximity_sensors_handles = [0] * 8

        # manejar los sensores de proximidad
        for i in range(len(self.proximity_sensors)):
            err_code, self.proximity_sensors_handles[i] = vrep.simxGetObjectHandle(
                self.clientID, "Proximity_sensor_" + self.proximity_sensors[i], vrep.simx_opmode_blocking)

        # inicialización de sensores de proximidad
        for sensor_name, sensor_handle in zip(self.proximity_sensors, self.proximity_sensors_handles):
            err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                self.clientID, sensor_handle, vrep.simx_opmode_streaming)

        self.distances_history = []

    def connect(self):
        vrep.simxFinish(-1)
        client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if client_id != -1:
            print("Conectado al servidor remoto API")
        else:
            print("No conectado al servidor remoto API")
            sys.exit("Podria no conectarse")

        return client_id

    def stop(self):
        #configurar el modo de parada
        force =0
        err_code = vrep.simxSetJointForce(self.clientID, self.left_front_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.left_back_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.right_back_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.right_front_handle, force, vrep.simx_opmode_oneshot)
        
        force =10
        for h in self.side_handles:
            err_code = vrep.simxSetJointForce(self.clientID, h, force, vrep.simx_opmode_oneshot)
        
        #descanso
        self.leftvelocity=10
        self.rightvelocity=10
        vrep.simxSetJointTargetVelocity(self.clientID,self.left_front_handle,self.leftvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.left_back_handle,self.leftvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.right_back_handle,self.rightvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.right_front_handle,self.rightvelocity,vrep.simx_opmode_streaming)
    
    def go(self):
        #configurar el modo de arranque
        force =10
        err_code = vrep.simxSetJointForce(self.clientID, self.left_front_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.left_back_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.right_back_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.right_front_handle, force, vrep.simx_opmode_oneshot)
        
        force =0
        for h in self.side_handles:
            err_code = vrep.simxSetJointForce(self.clientID, h, force, vrep.simx_opmode_oneshot)
    
    def setVelocity(self):
        #Verificar si la velocidad esta dentro del rango
        if self.leftvelocity > self.MaxVel:
            self.leftvelocity = self.MaxVel
        if self.leftvelocity < -self.MaxVel:
            self.leftvelocity = -self.MaxVel
        if self.rightvelocity > self.MaxVel:
            self.rightvelocity = self.MaxVel
        if self.rightvelocity < -self.MaxVel:
            self.rightvelocity = -self.MaxVel
        
        #Enviar valor de velocidad
        #vrep.simxSetJointTargetVelocity(clientID,left_front_handle,leftvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.left_back_handle,self.leftvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.right_back_handle,self.rightvelocity,vrep.simx_opmode_streaming)
        #vrep.simxSetJointTargetVelocity(clientID,right_front_handle,rightvelocity,vrep.simx_opmode_streaming)
    
    #Mueve el tanque hacia adelante
     #Ninguno: aumenta la velocidad en 1, si las velocidades de las ruedas son diferentes, se igualan
     #velocity: toma valores de <-10,10> y los establece como velocidad para ambas ruedas en dirección hacia adelante
    def forward(self, velocity=None):
        self.go()
        if velocity!=None:
            self.leftvelocity=velocity
            self.rightvelocity=velocity
        else:
            self.rightvelocity=self.leftvelocity=(self.leftvelocity+self.rightvelocity)/2
            self.leftvelocity+=self.dVel
            self.rightvelocity+=self.dVel
        self.setVelocity()
    
    #Mueve el tanque hacia atras
     #Ninguno: aumenta la velocidad en 1, si las velocidades de las ruedas son diferentes, se igualan
     #velocity: toma valores de <-10,10> y los establece como velocidad para ambas ruedas en dirección hacia adelante
    def backward(self, velocity=None):
        self.go()
        if velocity!=None:
            self.leftvelocity=-velocity
            self.rightvelocity=-velocity
        else:
            self.rightvelocity=self.leftvelocity=(self.leftvelocity+self.rightvelocity)/2
            self.leftvelocity-=self.dVel
            self.rightvelocity-=self.dVel
        self.setVelocity()
    
    #Gira a la izquierda el tanque
     #Ninguno - aumenta la velocidad de la rueda derecha en 1, disminuye la velocidad de la rueda izquierda en 1
     #velocity: toma valores de <-10,10> y los establece como velocidad para la rueda derecha
         #en dirección hacia adelante y valor opuesto de velocidad para la rueda izquierda en dirección hacia atrás
    def turn_left(self, velocity=None):
        self.go()
        if velocity!=None:
            self.leftvelocity =-velocity
            self.rightvelocity= velocity
        else:
            self.leftvelocity -=self.dVel
            self.rightvelocity+=self.dVel
        self.setVelocity()

    def turn_left_circle(self, velocity):
        self.go()
        self.leftvelocity = 10 * (velocity / 10)
        self.rightvelocity = 3 * (velocity / 10)
        self.setVelocity()


    def turn_right_circle(self, velocity):
        self.go()
        self.rightvelocity = 10 * (velocity / 10)
        self.leftvelocity = 3 * (velocity / 10)
        self.setVelocity()
    
    #Gira a la derecha el tanque
     #Ninguno - aumenta la velocidad de la rueda izquierda en 1, disminuye la velocidad de la rueda derecha en 1
     #velocity: toma valores de <-10,10> y los establece como velocidad para la rueda izquierda
         #en dirección hacia adelante y valor opuesto de velocidad para la rueda derecha en dirección hacia atrás
    def turn_right(self, velocity=None):
        self.go()
        if velocity!=None:
            self.leftvelocity = velocity
            self.rightvelocity=-velocity
        else:
            self.leftvelocity +=self.dVel
            self.rightvelocity-=self.dVel
        self.setVelocity()

    def read_distances(self):
        distances = []
        for sensor_name, sensor_handle in zip(self.proximity_sensors, self.proximity_sensors_handles):
            err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(self.clientID, sensor_handle, vrep.simx_opmode_buffer)
            norm = np.linalg.norm(detectedPoint)
            distance = norm if err_code == 0 and norm > 1e-2 else math.inf
            distance2 = distance if detectionState else math.inf
            distances.extend([distance, distance2])

        distances = Distances(*distances)
        self.distances_history.append(distances)
        return distances

    def restart_plot(self):
        self.distances_history.clear()

    def plot_distances(self):
        x = np.arange(len(self.distances_history)) / 10

        nw = [d.nw for d in self.distances_history]
        ne = [d.ne for d in self.distances_history]
        wn = [d.wn for d in self.distances_history]
        en = [d.en for d in self.distances_history]
        sw = [d.sw for d in self.distances_history]
        se = [d.se for d in self.distances_history]
        ws = [d.ws for d in self.distances_history]
        es = [d.es for d in self.distances_history]

        plt.plot(x, nw, '-', label='nw')
        plt.plot(x, ne, '-', label='ne')
        plt.plot(x, wn, '-', label='wn')
        plt.plot(x, en, '-', label='en')
        plt.plot(x, sw, '--', label='sw')
        plt.plot(x, se, '--', label='se')
        plt.plot(x, ws, '--', label='ws')
        plt.plot(x, es, '--', label='es')

        plt.legend()
        plt.show()
