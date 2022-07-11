# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GUI_designer_pyqtgraph.ui'
#
# Created by: PyQt5 UI code generator 5.12
#
# WARNING! All changes made in this file will be lost!
from PyQt5 import QtCore, QtGui, QtWidgets
import xsensdeviceapi as xda
from PyQt5.QtCore import QThread
import pyquaternion as q
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import pyqtgraph as pg
import matplotlib.pyplot as plt
import sys
from datetime import datetime
import csv
import scipy.signal
from threading import Lock


class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size = 5):
        xda.XsCallback.__init__(self)
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = list()
        self.m_lock = Lock()

    def packetAvailable(self):
        self.m_lock.acquire()
        res = len(self.m_packetBuffer) > 0
        self.m_lock.release()
        return res

    def getNextPacket(self):
        self.m_lock.acquire()
        assert(len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        self.m_lock.release()
        return oldest_packet

    def onLiveDataAvailable(self, dev, packet):
        self.m_lock.acquire()
        assert(packet is not 0)
        while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer.pop()
        self.m_packetBuffer.append(xda.XsDataPacket(packet))
        self.m_lock.release()



class Reader_thread(QtCore.QThread):
    data = QtCore.pyqtSignal(np.ndarray,np.ndarray,list,np.ndarray,np.ndarray)
    def __init__(self,parent=None,callbacks=[],main_callback=None,device_ids=[]):
        super(Reader_thread, self).__init__(parent)
        self.callbacks=callbacks
        self.device_ids=device_ids
        self.main_callback=main_callback
        self.counter=0
        self.xsens_acceleration_data=np.zeros((6,3))
        self.xsens_quaternion_data=np.zeros((6,4))
        self.xsens_time_data=np.zeros(6)
        self.init_quats=[]
        self.time_start=time.time()
        self.should_send=6
        self.data_counter=np.zeros(6)
        self.reset_quats=np.zeros(6)
        self.xsens_quaternion_list=[[] for i in range(6)]
        self.xsens_acceleration_list=[[] for i in range(6)]
        self.xsens_time_list=[[] for i in range(6)]
        self.xsens_gyro_list=[[] for i in range(6)]
        for i in range(6):
            self.init_quats.append(q.Quaternion(0,0,0,0))
    def run(self):
        while 1:
            while len(self.main_callback.m_packetBuffer)>6:
                for i in range(6):
                    # while not self.callbacks[i].packetAvailable() :
                    #     pass
                        
                    
                    packet = self.main_callback.getNextPacket()
                    ind=self.device_ids.index(packet.deviceId().toXsString())
                    # print(packet.deviceId())
                    if(self.init_quats[i]==q.Quaternion(0,0,0,0) or self.reset_quats[i]):
                        self.reset_quats[i]=0
                        self.init_quats[i]=q.Quaternion(np.array(packet.orientationQuaternion()))
                    # print(packet.estimatedTimeOfSampling().msTime())
                    # self.xsens_quaternion_data[i,1:1000,:]=self.xsens_quaternion_data[i,0:999,:]
                    # self.xsens_quaternion_data[i,0,:]=np.array(packet.orientationQuaternion())
                    self.xsens_quaternion_list[ind].append(np.array(packet.orientationQuaternion()))
                    # self.xsens_quaternion_data[i]=np.array(packet.orientationQuaternion())
                    self.xsens_acceleration_list[ind].append(np.reshape(np.array(packet.calibratedAcceleration()),(3,)))
                    self.xsens_gyro_list[ind].append(np.reshape(np.array(packet.calibratedGyroscopeData()),(3,)))
                    # packet_acceleration=np.reshape(np.array(packet.calibratedAcceleration()),(3,))
                    # self.xsens_acceleration_data[i]=packet_acceleration
                    self.counter=self.counter+1
                    self.data_counter[ind]+=1
                    # self.xsens_time_data[i]=packet.estimatedTimeOfSampling().msTime()
                    self.xsens_time_list[ind].append(packet.estimatedTimeOfSampling().msTime())
                    if(len(self.xsens_quaternion_list[ind][0])!=len(set(self.xsens_quaternion_list[ind][0]))):
                        print('greska')
                while (self.data_counter>0.1).all():
                    try:
                        self.data.emit(np.asarray([e.pop(0) for e in self.xsens_quaternion_list]),np.asarray([e.pop(0) for e in self.xsens_acceleration_list]),self.init_quats,np.asarray([e.pop(0) for e in self.xsens_time_list]),np.asarray([e.pop(0) for e in self.xsens_gyro_list]))
                        self.data_counter-=1
                    except:
                        print('corrupted_packet')
                        
                for i in range(6):
                    while(len(self.xsens_quaternion_list[i])>10 and len(self.xsens_acceleration_list[i])>10 and len(self.xsens_time_list[i])>10 and len(self.xsens_gyro_list[i])>10):
                        self.xsens_quaternion_list[i].pop(0)
                        self.xsens_acceleration_list[i].pop(0)
                        self.xsens_time_list[i].pop(0)
                        self.xsens_gyro_list[i].pop(0)
                        self.data_counter[i]-=1
            
            self.usleep(100)
    def clear_init_quats(self):
        self.reset_quats+=1


def calculate_stride(acceleration, foot_timestamps,same_shank_timestamps, opposite_shank_timestamps, stride_indices, drift_indices,is_last):
    # promenljive prosledjene po prirodnom vremenskom poretku - obrnuto od cuvanja
    # konvertovanje vremenskih obelezja
    timestamp_increments=np.diff(foot_timestamps)
    timestamp_increments=np.round(timestamp_increments/10)
    discrete_timestamps=np.array([0]+list(np.cumsum(timestamp_increments)))

    synced_stride_indices=stride_indices
    synced_stride_indices[0]=np.argmin(np.abs(same_shank_timestamps[synced_stride_indices[0]]-foot_timestamps))
    if not is_last: 
        synced_stride_indices[1]=np.argmin(np.abs(same_shank_timestamps[synced_stride_indices[1]]-foot_timestamps))

    synced_drift_indices=drift_indices
    synced_drift_indices[0]=np.argmin(np.abs(opposite_shank_timestamps[synced_drift_indices[0]]-foot_timestamps))
    synced_drift_indices[1]=np.argmin(np.abs(opposite_shank_timestamps[synced_drift_indices[1]]-foot_timestamps))
    if not is_last:
        synced_drift_indices[2]=np.argmin(np.abs(opposite_shank_timestamps[synced_drift_indices[2]]-foot_timestamps))

    stride_index1=int(discrete_timestamps[synced_stride_indices[0]])
    stride_index2=int(discrete_timestamps[synced_stride_indices[1]])

    drift_index1=int(discrete_timestamps[synced_drift_indices[0]])
    drift_index2=int(discrete_timestamps[synced_drift_indices[1]])
    drift_index3=int(discrete_timestamps[synced_drift_indices[2]])

    interp_acc=np.interp(np.arange(0,discrete_timestamps[-1]+1),discrete_timestamps,acceleration)
    velocity=np.cumsum(interp_acc)/100
    velocity[drift_index1:drift_index2]-=np.linspace(velocity[drift_index1],velocity[drift_index2-1],len(velocity[drift_index1:drift_index2]))
    velocity[drift_index2:drift_index3]-=np.linspace(velocity[drift_index2],velocity[drift_index3-1],len(velocity[drift_index2:drift_index3]))

    return np.sum(velocity[stride_index1:stride_index2])/100

class Processing_thread(QtCore.QThread):
    processed_data=QtCore.pyqtSignal(np.ndarray,np.ndarray,float,float,np.ndarray)
    def __init__(self,parent=None,device_ids=[],angles_left_list_model=None,angles_right_list_model=None, stride_duration_left_list_model=None,stride_duration_right_list_model=None,stride_length_left_list_model=None,stride_length_right_list_model=None):
                        
        super(Processing_thread, self).__init__(parent)
        self.counter=0
        self.xsens_orientation_data=np.zeros((6,3))
        self.xsens_acceleration_data=[]
        self.xsens_velocity_data=[]
        self.xsens_quaternion_data=[]
        self.xsens_gyro_data=[]
        self.init_quats=[]
        self.xsens_gyro=[]
        self.oriented_acceleration_data=np.zeros((6,3))
        self.previous_oriented_acceleration_data=np.zeros((6,3))
        self.ids=device_ids
        self.reccording=False

        self.xsens_marker_right=0
        self.xsens_marker_left=0
        self.xsens_marker_left_array=np.zeros(1000)
        self.xsens_marker_right_array=np.zeros(1000)
        self.tracking_markers=False
        self.tracking_markers_old=False
        self.angle_trend_left_1=0
        self.angle_trend_left_2=0
        self.angle_trend_right_1=0
        self.angle_trend_right_2=0
        self.left_marker_distance=0
        self.right_marker_distance=0
        self.previous_angle_left=0
        self.previous_angle_right=0
        self.state_left=True
        self.state_right=True
        self.timestamps=[]
        self.calculate_stride_left=0
        self.calculate_stride_right=0
        self.left_trajectory=0
        self.right_trajectory=0
        self.b_filter,self.a_filter=scipy.signal.butter(3,0.2) 
        self.xsens_orientation_data_array=np.zeros((6,1000,3))
        self.xsens_acceleration_data_array=np.zeros((6,1000,3))
        self.xsens_gyro_data_array=np.zeros((6,1000,3))
        self.additional_left_stride=0
        self.additional_right_stride=0
        self.timestamp_array=np.zeros((6,1000))

        self.angles_left_list_model=angles_left_list_model

        self.angles_right_list_model=angles_right_list_model

        self.stride_duration_left_list_model=stride_duration_left_list_model

        self.stride_duration_right_list_model=stride_duration_right_list_model

        self.stride_length_left_list_model=stride_length_left_list_model

        self.stride_length_right_list_model=stride_length_right_list_model

    def print_log(self,model,string):
        item=QtGui.QStandardItem(string)
        model.insertRow(0,item)
        QtWidgets.qApp.processEvents()
    def run(self):
        while 1:
            while len(self.xsens_quaternion_data)>0:
                right_thigh_index=self.ids.index('00B4437D')
                quotient=q.Quaternion(self.xsens_quaternion_data[0][right_thigh_index])
                rotated_vector=quotient.rotate([0,1,0])
                current_angle_transversal=180/np.pi*np.arctan2(rotated_vector[1],rotated_vector[0])
                reverse_direction=False
                if (180-np.abs(current_angle_transversal))<90:
                    reverse_direction=True

                for i in range(6):
                    quotient=q.Quaternion(self.xsens_quaternion_data[0][i])

                    if reverse_direction:
                        quotient=q.Quaternion([0,0,0,-1])*q.Quaternion(self.xsens_quaternion_data[0][i])
                    if self.ids[i]=='00B44384':
                        rotated_vector=quotient.rotate([1,0,0])
                        current_angle_sagital=180/np.pi*np.arctan2(rotated_vector[2],rotated_vector[0])

                        rotated_vector=quotient.rotate([0,0,1])
                        current_angle_frontal=-180/np.pi*np.arctan2(rotated_vector[2],rotated_vector[1])

                        rotated_vector=quotient.rotate([0,1,0])
                        current_angle_transversal=-180/np.pi*np.arctan2(rotated_vector[0],rotated_vector[1])
                    elif self.ids[i]=='00B44389':
                        rotated_vector=quotient.rotate([1,0,0])
                        current_angle_sagital=180/np.pi*np.arctan2(rotated_vector[2],rotated_vector[0])

                        rotated_vector=quotient.rotate([0,0,1])
                        current_angle_frontal=-180/np.pi*np.arctan2(rotated_vector[2],-rotated_vector[1])

                        rotated_vector=quotient.rotate([0,1,0])
                        current_angle_transversal=180/np.pi*np.arctan2(rotated_vector[0],rotated_vector[1])
                    elif self.ids[i]=='00B443B5' or self.ids[i]=='00B4437D':

                        rotated_vector=quotient.rotate([1,0,0])
                        current_angle_sagital=180/np.pi*np.arctan2(rotated_vector[0],-rotated_vector[2])

                        rotated_vector=quotient.rotate([0,0,1])
                        current_angle_frontal=180/np.pi*np.arctan2(rotated_vector[2],-rotated_vector[1])

                        rotated_vector=quotient.rotate([0,1,0])
                        current_angle_transversal=-180/np.pi*np.arctan2(rotated_vector[1],rotated_vector[0])
                    else:
                        rotated_vector=quotient.rotate([1,0,0])
                        current_angle_sagital=180/np.pi*np.arctan2(rotated_vector[0],-rotated_vector[2])

                        rotated_vector=quotient.rotate([0,0,1])
                        current_angle_frontal=-180/np.pi*np.arctan2(-rotated_vector[2],rotated_vector[1])

                        rotated_vector=quotient.rotate([0,1,0])
                        current_angle_transversal=-180/np.pi*np.arctan2(rotated_vector[1],-rotated_vector[0])

                    # if  self.ids[i] (180-np.abs(current_angle_transversal))<90:
                    #     current_angle_sagital=-current_angle_sagital
                    self.xsens_orientation_data[i,:]=np.asarray([current_angle_sagital,current_angle_frontal,current_angle_transversal])
                    
                    packet_acceleration=self.xsens_acceleration_data[0][i]
                    oriented_acc=quotient.inverse.rotate(packet_acceleration.tolist())
                    rotated_vectorx=quotient.inverse.rotate([1,0,0])
                    rotated_vectory=quotient.inverse.rotate([0,1,0])
                    rotated_vectorz=quotient.inverse.rotate([0,0,1])
                    path_acceleration=np.matmul(np.reshape(np.asarray(rotated_vectorx),(1,3)),packet_acceleration)
                    normal_acceleration=np.matmul(np.reshape(np.asarray(rotated_vectory),(1,3)),packet_acceleration)
                    vertical_acceleration=np.matmul(np.reshape(np.asarray(rotated_vectorz),(1,3)),packet_acceleration)
                    # path_acceleration=[oriented_acc[0]]
                    # normal_acceleration=[oriented_acc[1]]
                    # vertical_acceleration=[oriented_acc[2]]
                    alpha=0
                    a=alpha
                    b=1-a

                    self.oriented_acceleration_data[i]=b*np.concatenate([path_acceleration,normal_acceleration,vertical_acceleration]).T+a*self.previous_oriented_acceleration_data[i]
                    self.previous_oriented_acceleration_data[i]=self.oriented_acceleration_data[i]
                    
                self.xsens_marker_right=0
                self.xsens_marker_left=0  
                self.left_stride=0
                self.right_stride=0 
                
                if self.tracking_markers:
                    if not self.tracking_markers_old:
                        self.xsens_marker_right=1000
                        self.xsens_marker_left=1000
                        self.calculate_stride_right=0
                        self.calculate_stride_left=0
                        self.left_trajectory=0
                        self.right_trajectory=0
                        print('______________________________')
                    else:
                        left_gyro=self.xsens_gyro_data_array[self.ids.index('00B44396'),:,2]
                        left_gyro=-left_gyro
                        right_gyro=self.xsens_gyro_data_array[self.ids.index('00B4437D'),:,2]
                        state_left=left_gyro>0
                        state_right=right_gyro>0
                        i=21
                        old_state_left=np.mean(state_left[i-20:i+20])>0.5
                        old_state_right=np.mean(state_right[i-20:i+20])>0.5
                        i=20
                        new_state_left=np.mean(state_left[i-20:i+20])>0.5
                        new_state_right=np.mean(state_right[i-20:i+20])>0.5
                        if new_state_left and not old_state_left and np.std(self.xsens_gyro_data_array[self.ids.index('00B44396'),:np.where(self.xsens_marker_left_array)[0][0],0])>0.1/(1+0*(np.sum(self.xsens_marker_left_array)==1)):
                            new_state_left=state_left*1
                            for i in range(len(left_gyro)):
                                if i+20<len(left_gyro) and i>=20:
                                    new_state_left[i]=np.mean(state_left[i-20:i+20])>0.5
                            self.xsens_marker_left=np.argmin(left_gyro[20:21+int(np.where(new_state_left[21:])[0][0]/3)])+20
                            # while(left_gyro[ self.xsens_marker_left]<0):
                            #     self.xsens_marker_left-=1
                            if np.sum(self.xsens_marker_left_array)>1:
                                if np.std(self.xsens_gyro_data_array[self.ids.index('00B44396'),0:40,0])<0.1 or np.std(np.sqrt(self.xsens_acceleration_data_array[self.ids.index('00B44384'),0:40,0]**2+self.xsens_acceleration_data_array[self.ids.index('00B44384'),0:40,1]**2+self.xsens_acceleration_data_array[self.ids.index('00B44384'),0:40,2]**2))<0.1:
                                    self.xsens_marker_left=0
                                else:
                                    self.calculate_stride_left+=1
                        if new_state_right and not old_state_right and np.std(self.xsens_gyro_data_array[self.ids.index('00B4437D'),:np.where(self.xsens_marker_right_array)[0][0],0])>0.1/(1+0*(np.sum(self.xsens_marker_right_array)==1)):
                            new_state_right=state_left*1
                            for i in range(len(right_gyro)):
                                if i+20<len(right_gyro) and i>=20:
                                    new_state_right[i]=np.mean(state_right[i-20:i+20])>0.5
                            self.xsens_marker_right=np.argmin(right_gyro[20:21+int(np.where(new_state_right[21:])[0][0]/3)])+20
                            # while(right_gyro[self.xsens_marker_right]<0):
                            #     self.xsens_marker_right-=1
                            if np.sum(self.xsens_marker_right_array)>1:
                                if np.std(self.xsens_gyro_data_array[self.ids.index('00B4437D'),0:40,0])<0.1 or np.std(np.sqrt(self.xsens_acceleration_data_array[self.ids.index('00B44389'),0:40,0]**2+self.xsens_acceleration_data_array[self.ids.index('00B44389'),0:40,1]**2+self.xsens_acceleration_data_array[self.ids.index('00B44389'),0:40,2]**2))<0.1:
                                    self.xsens_marker_right=0
                                else:
                                    self.calculate_stride_right+=1

                        if self.calculate_stride_left:
                            right_gyro=self.xsens_gyro_data_array[self.ids.index('00B4437D'),:,2]
                            i=21
                            old_state_right=np.mean(state_right[i-20:i+20])>0.5
                            i=20
                            new_state_right=np.mean(state_right[i-20:i+20])>0.5
                            if  new_state_right and not old_state_right :
                                if np.sum(self.xsens_marker_left_array)>=3:
                                    index1=20
                                    index2=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1]])+np.where(self.xsens_marker_left_array)[0][0]
                                    index3=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][1]:np.where(self.xsens_marker_left_array)[0][2]])+np.where(self.xsens_marker_left_array)[0][1]
                                    velocity_left=np.cumsum(scipy.signal.filtfilt(self.b_filter,self.a_filter,np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0])))/100
                                    index1=1000-index3
                                    index2=1000-index2
                                    index3=1000-20

                                    velocity_left[index1:index2]=velocity_left[index1:index2]-np.linspace(velocity_left[index1],velocity_left[index2],len(velocity_left[index1:index2]))
                                    velocity_left[index2:index3]=velocity_left[index2:index3]-np.linspace(velocity_left[index2],velocity_left[index3],len(velocity_left[index2:index3]))
                                    velocity_interp_before=velocity_left[1000-np.where(self.xsens_marker_left_array)[0][1]:1000-np.where(self.xsens_marker_left_array)[0][0]]
                                    timestamps=np.flip(self.timestamp_array[self.ids.index('00B44384'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1]])
                                    timestamps=timestamps-timestamps[0]
                                    timestamp_increments=np.round((np.diff(timestamps)/10))
                                    timestamp_interpolate=np.array([0]+list(np.cumsum(timestamp_increments)))
                                    velocity_interp=np.interp(np.arange(0,len(timestamp_interpolate)),timestamp_interpolate,velocity_interp_before)



                                    # self.left_stride=np.sum(velocity_interp)/100
                                    self.left_stride_x= calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0]), np.flip(self.timestamp_array[self.ids.index('00B44384'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]),[1000-np.where(self.xsens_marker_left_array)[0][1],1000-np.where(self.xsens_marker_left_array)[0][0]],[index1,index2,index3],False)
                                    self.left_stride_y = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,1]), np.flip(self.timestamp_array[self.ids.index('00B44384'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]),[1000-np.where(self.xsens_marker_left_array)[0][1],1000-np.where(self.xsens_marker_left_array)[0][0]],[index1,index2,index3],False)
                                    self.left_stride=np.sqrt(self.left_stride_x**2+self.left_stride_y**2)
                                    print('Left stride:',self.left_stride)
                                    self.left_trajectory+=self.left_stride

                                    self.print_log(self.angles_left_list_model,'______')
                                    self.print_log(self.angles_left_list_model,'F: '+ str(int(np.max(self.xsens_orientation_data_array[self.ids.index('00B44384'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1],0])))+'  '+str( int(np.min(self.xsens_orientation_data_array[self.ids.index('00B44384'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1],0]))))
                                    self.print_log(self.angles_left_list_model,'S: '+ str(int(np.max(self.xsens_orientation_data_array[self.ids.index('00B44396'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1],0])))+'  '+str( int(np.min(self.xsens_orientation_data_array[self.ids.index('00B44396'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1],0]))))
                                    self.print_log(self.angles_left_list_model,'T: '+ str(int(np.max(self.xsens_orientation_data_array[self.ids.index('00B443B2'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1],0])))+'  '+str( int(np.min(self.xsens_orientation_data_array[self.ids.index('00B443B2'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1],0]))))

                                    self.print_log(self.stride_duration_left_list_model,'______')
                                    self.print_log(self.stride_duration_left_list_model,"%.2f" %((np.where(self.xsens_marker_left_array)[0][1]-np.where(self.xsens_marker_left_array)[0][0])/100))


                                    self.print_log(self.stride_length_left_list_model,'______')
                                    self.print_log(self.stride_length_left_list_model,"%.2f" %self.left_stride)

                                    self.calculate_stride_left-=1
                                else:
                                    index1=20
                                    index2=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1]])+np.where(self.xsens_marker_left_array)[0][0]
                                    index3=np.where(self.xsens_marker_left_array)[0][2]
                                    velocity_left=np.cumsum(scipy.signal.filtfilt(self.b_filter,self.a_filter,np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0])))/100
                                    while np.std(np.sqrt(self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,0]**2+self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,1]**2+self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,2]**2))<0.1:
                                        index3-=1

                                    index1=1000-index3
                                    index2=1000-index2
                                    index3=1000-20

                                    velocity_left[index1:index2]=velocity_left[index1:index2]-np.linspace(velocity_left[index1],velocity_left[index2],len(velocity_left[index1:index2]))
                                    velocity_left[index2:index3]=velocity_left[index2:index3]-np.linspace(velocity_left[index2],velocity_left[index3],len(velocity_left[index2:index3]))
                                    

                                    velocity_interp_before=velocity_left[1000-np.where(self.xsens_marker_left_array)[0][1]:1000-np.where(self.xsens_marker_left_array)[0][0]]
                                    timestamps=np.flip(self.timestamp_array[self.ids.index('00B44384'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1]])
                                    timestamps=timestamps-timestamps[0]
                                    timestamp_increments=np.round((np.diff(timestamps)/10))
                                    timestamp_interpolate=np.array([0]+list(np.cumsum(timestamp_increments)))
                                    velocity_interp=np.interp(np.arange(0,len(timestamp_interpolate)),timestamp_interpolate,velocity_interp_before)


                                    self.left_stride_x = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0]), np.flip(self.timestamp_array[self.ids.index('00B44384'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]) ,[1000-np.where(self.xsens_marker_left_array)[0][1],1000-np.where(self.xsens_marker_left_array)[0][0]],[index1,index2,index3],False)
                                    self.left_stride_y = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,1]), np.flip(self.timestamp_array[self.ids.index('00B44384'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]) ,[1000-np.where(self.xsens_marker_left_array)[0][1],1000-np.where(self.xsens_marker_left_array)[0][0]],[index1,index2,index3],False)
                                    self.left_stride=np.sqrt(self.left_stride_x**2+self.left_stride_y**2)

                                    # self.left_stride=np.sum(velocity_interp)/100
                                    print('Left stride:',self.left_stride)
                                    self.left_trajectory+=self.left_stride
                                    self.calculate_stride_left-=1




                        if self.calculate_stride_right:    
                            left_gyro=self.xsens_gyro_data_array[self.ids.index('00B44396'),:,2]
                            left_gyro=-left_gyro
                            i=21
                            old_state_left=np.mean(state_left[i-20:i+20])>0.5
                            i=20
                            new_state_left=np.mean(state_left[i-20:i+20])>0.5
                            if   new_state_left and not old_state_left :
                                if np.sum(self.xsens_marker_right_array)>3:
                                    index1=20
                                    index2=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1]])+np.where(self.xsens_marker_right_array)[0][0]
                                    index3=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][1]:np.where(self.xsens_marker_right_array)[0][2]])+np.where(self.xsens_marker_right_array)[0][1]
                                    velocity_right=np.cumsum(scipy.signal.filtfilt(self.b_filter,self.a_filter,np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0])))/100
                                    index1=1000-index3
                                    index2=1000-index2
                                    index3=1000-20

                                    velocity_right[index1:index2]=velocity_right[index1:index2]-np.linspace(velocity_right[index1],velocity_right[index2],len(velocity_right[index1:index2]))
                                    velocity_right[index2:index3]=velocity_right[index2:index3]-np.linspace(velocity_right[index2],velocity_right[index3],len(velocity_right[index2:index3]))
                                    
                                    velocity_interp_before=velocity_right[1000-np.where(self.xsens_marker_right_array)[0][1]:1000-np.where(self.xsens_marker_right_array)[0][0]]
                                    timestamps=np.flip(self.timestamp_array[self.ids.index('00B44389'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1]])
                                    timestamps=timestamps-timestamps[0]
                                    timestamp_increments=np.round((np.diff(timestamps)/10))
                                    timestamp_interpolate=np.array([0]+list(np.cumsum(timestamp_increments)))
                                    velocity_interp=np.interp(np.arange(0,len(timestamp_interpolate)),timestamp_interpolate,velocity_interp_before)


                                    self.right_stride_x = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0]), np.flip(self.timestamp_array[self.ids.index('00B44389'),:]),  np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]),[1000-np.where(self.xsens_marker_right_array)[0][1],1000-np.where(self.xsens_marker_right_array)[0][0]],[index1,index2,index3],False)
                                    self.right_stride_y = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,1]), np.flip(self.timestamp_array[self.ids.index('00B44389'),:]),  np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]),[1000-np.where(self.xsens_marker_right_array)[0][1],1000-np.where(self.xsens_marker_right_array)[0][0]],[index1,index2,index3],False)
                                    self.right_stride=np.sqrt(self.right_stride_x**2+self.right_stride_y**2)
                                    # self.right_stride=np.sum(velocity_interp)/100
                                    print('Right stride:',self.right_stride)

                                    self.print_log(self.angles_right_list_model,'______')
                                    self.print_log(self.angles_right_list_model,'F: '+ str(int(np.max(self.xsens_orientation_data_array[self.ids.index('00B44389'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1],0])))+'  '+str( int(np.min(self.xsens_orientation_data_array[self.ids.index('00B44389'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1],0]))))
                                    self.print_log(self.angles_right_list_model,'S: '+ str(int(np.max(self.xsens_orientation_data_array[self.ids.index('00B4437D'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1],0])))+'  '+str( int(np.min(self.xsens_orientation_data_array[self.ids.index('00B4437D'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1],0]))))
                                    self.print_log(self.angles_right_list_model,'T: '+ str(int(np.max(self.xsens_orientation_data_array[self.ids.index('00B443B5'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1],0])))+'  '+str( int(np.min(self.xsens_orientation_data_array[self.ids.index('00B443B5'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1],0]))))

                                    self.print_log(self.stride_duration_right_list_model,'______')
                                    self.print_log(self.stride_duration_right_list_model,"%.2f" %((np.where(self.xsens_marker_right_array)[0][1]-np.where(self.xsens_marker_right_array)[0][0])/100))


                                    self.print_log(self.stride_length_right_list_model,'______')
                                    self.print_log(self.stride_length_right_list_model,"%.2f" %self.right_stride)



                                    self.right_trajectory+=self.right_stride
                                    self.calculate_stride_right-=1



                                    
                                else:
                                    index1=20
                                    index2=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1]])+np.where(self.xsens_marker_right_array)[0][0]
                                    index3=np.where(self.xsens_marker_right_array)[0][2]
                                    while np.std(np.sqrt(self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,0]**2+self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,1]**2+self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,2]**2))<0.1:
                                        index3-=1
                                    velocity_right=np.cumsum(scipy.signal.filtfilt(self.b_filter,self.a_filter,np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0])))/100
                                    index1=1000-index3
                                    index2=1000-index2
                                    index3=1000-20

                                    velocity_right[index1:index2]=velocity_right[index1:index2]-np.linspace(velocity_right[index1],velocity_right[index2],len(velocity_right[index1:index2]))
                                    velocity_right[index2:index3]=velocity_right[index2:index3]-np.linspace(velocity_right[index2],velocity_right[index3],len(velocity_right[index2:index3]))
                                    


                                    velocity_interp_before=velocity_right[1000-np.where(self.xsens_marker_right_array)[0][1]:1000-np.where(self.xsens_marker_right_array)[0][0]]
                                    timestamps=np.flip(self.timestamp_array[self.ids.index('00B44389'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1]])
                                    timestamps=timestamps-timestamps[0]
                                    timestamp_increments=np.round((np.diff(timestamps)/10))
                                    timestamp_interpolate=np.array([0]+list(np.cumsum(timestamp_increments)))
                                    velocity_interp=np.interp(np.arange(0,len(timestamp_interpolate)),timestamp_interpolate,velocity_interp_before)


                                    self.right_stride_x = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0]), np.flip(self.timestamp_array[self.ids.index('00B44389'),:]),  np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]),np.flip(self.timestamp_array[self.ids.index('00B44396'),:]) ,[1000-np.where(self.xsens_marker_right_array)[0][1],1000-np.where(self.xsens_marker_right_array)[0][0]],[index1,index2,index3],False)
                                    self.right_stride_y = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,1]), np.flip(self.timestamp_array[self.ids.index('00B44389'),:]),  np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]),np.flip(self.timestamp_array[self.ids.index('00B44396'),:]) ,[1000-np.where(self.xsens_marker_right_array)[0][1],1000-np.where(self.xsens_marker_right_array)[0][0]],[index1,index2,index3],False)
                                    self.right_stride=np.sqrt(self.right_stride_x**2+self.right_stride_y**2)
                                    # self.right_stride=np.sum(velocity_interp)/100
                                    print('Right stride:',self.right_stride)

                                    



                                    self.right_trajectory+=self.right_stride
                                    self.calculate_stride_right-=1






                if self.tracking_markers_old and not self.tracking_markers:
                    self.xsens_marker_right=1000
                    self.xsens_marker_left=1000
                    if self.calculate_stride_left and len(np.where(self.xsens_marker_left_array)[0])>=3:
                        right_gyro=self.xsens_gyro_data_array[self.ids.index('00B4437D'),:,2]
                        index1=20
                        index2=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1]])+np.where(self.xsens_marker_left_array)[0][0]
                        index3=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][1]:np.where(self.xsens_marker_left_array)[0][2]])+np.where(self.xsens_marker_left_array)[0][1]
                        velocity_left=np.cumsum(scipy.signal.filtfilt(self.b_filter,self.a_filter,np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0])))/100
                        index1=1000-index3
                        index2=1000-index2
                        index3=20
                        while np.std(np.sqrt(self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,0]**2+self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,1]**2+self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,2]**2))<0.1:
                            index3+=1
                        index3=1000-index3
                        velocity_left[index1:index2]=velocity_left[index1:index2]-np.linspace(velocity_left[index1],velocity_left[index2],len(velocity_left[index1:index2]))
                        velocity_left[index2:index3]=velocity_left[index2:index3]-np.linspace(velocity_left[index2],velocity_left[index3],len(velocity_left[index2:index3]))
                        
                        velocity_interp_before=velocity_left[1000-np.where(self.xsens_marker_left_array)[0][1]:1000-np.where(self.xsens_marker_left_array)[0][0]]
                        timestamps=np.flip(self.timestamp_array[self.ids.index('00B44384'),np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1]])
                        timestamps=timestamps-timestamps[0]
                        timestamp_increments=np.round((np.diff(timestamps)/10))
                        timestamp_interpolate=np.array([0]+list(np.cumsum(timestamp_increments)))
                        velocity_interp=np.interp(np.arange(0,len(timestamp_interpolate)),timestamp_interpolate,velocity_interp_before)

                        self.additional_left_stride_x = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0]), np.flip(self.timestamp_array[self.ids.index('00B44384'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]),[1000-np.where(self.xsens_marker_left_array)[0][1],1000-np.where(self.xsens_marker_left_array)[0][0]],[index1,index2,index3],False)
                        self.additional_left_stride_y = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,1]), np.flip(self.timestamp_array[self.ids.index('00B44384'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]),[1000-np.where(self.xsens_marker_left_array)[0][1],1000-np.where(self.xsens_marker_left_array)[0][0]],[index1,index2,index3],False)
                        self.additional_left_stride=np.sqrt(self.additional_left_stride_x**2+self.additional_left_stride_y**2)

                        # self.additional_left_stride=np.sum(velocity_interp)/100
                        print('Left stride:',self.additional_left_stride)
                        self.left_trajectory+=self.additional_left_stride
                        
                    if self.calculate_stride_right and len(np.where(self.xsens_marker_right_array)[0])>=3:   
                        left_gyro=self.xsens_gyro_data_array[self.ids.index('00B44396'),:,2]
                        index1=20
                        index2=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1]])+np.where(self.xsens_marker_right_array)[0][0]
                        index3=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][1]:np.where(self.xsens_marker_right_array)[0][2]])+np.where(self.xsens_marker_right_array)[0][1]
                        velocity_right=np.cumsum(scipy.signal.filtfilt(self.b_filter,self.a_filter,np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0])))/100
                        index1=1000-index3
                        index2=1000-index2
                        index3=20
                        while np.std(np.sqrt(self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,0]**2+self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,1]**2+self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,2]**2))<0.1:
                            index3+=1
                        index3=1000-index3
                        velocity_right[index1:index2]=velocity_right[index1:index2]-np.linspace(velocity_right[index1],velocity_right[index2],len(velocity_right[index1:index2]))
                        velocity_right[index2:index3]=velocity_right[index2:index3]-np.linspace(velocity_right[index2],velocity_right[index3],len(velocity_right[index2:index3]))
                        

                        velocity_interp_before=velocity_right[1000-np.where(self.xsens_marker_right_array)[0][1]:1000-np.where(self.xsens_marker_right_array)[0][0]]
                        timestamps=np.flip(self.timestamp_array[self.ids.index('00B44389'),np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1]])
                        timestamps=timestamps-timestamps[0]
                        timestamp_increments=np.round((np.diff(timestamps)/10))
                        timestamp_interpolate=np.array([0]+list(np.cumsum(timestamp_increments)))
                        velocity_interp=np.interp(np.arange(0,len(timestamp_interpolate)),timestamp_interpolate,velocity_interp_before)


                        self.additional_right_stride_x = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0]), np.flip(self.timestamp_array[self.ids.index('00B44389'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]),[1000-np.where(self.xsens_marker_right_array)[0][1],1000-np.where(self.xsens_marker_right_array)[0][0]],[index1,index2,index3],False)
                        self.additional_right_stride_y = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,1]), np.flip(self.timestamp_array[self.ids.index('00B44389'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]),[1000-np.where(self.xsens_marker_right_array)[0][1],1000-np.where(self.xsens_marker_right_array)[0][0]],[index1,index2,index3],False)
                        self.additional_right_stride=np.sqrt(self.additional_right_stride_x**2+self.additional_right_stride_y**2)
                        # self.additional_right_stride=np.sum(velocity_interp)/100
                        print('Right stride:',self.additional_right_stride)
                        self.right_trajectory+=self.additional_right_stride



                    if len(np.where(self.xsens_marker_left_array)[0])>=3:
                        right_gyro=self.xsens_gyro_data_array[self.ids.index('00B4437D'),:,2]
                        index1=20
                        index2=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1]])+np.where(self.xsens_marker_left_array)[0][0]
                        index3=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][1]:np.where(self.xsens_marker_left_array)[0][2]])+np.where(self.xsens_marker_left_array)[0][1]
                        velocity_left=np.cumsum(scipy.signal.filtfilt(self.b_filter,self.a_filter,np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0])))/100
                        index1=1000-index3
                        index2=1000-index2
                        index3=20
                        while np.std(np.sqrt(self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,0]**2+self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,1]**2+self.xsens_acceleration_data_array[self.ids.index('00B44384'),index3-20:index3+20,2]**2))<0.1:
                            index3+=1
                        index3=1000-index3
                        velocity_left[index1:index2]=velocity_left[index1:index2]-np.linspace(velocity_left[index1],velocity_left[index2],len(velocity_left[index1:index2]))
                        velocity_left[index2:index3]=velocity_left[index2:index3]-np.linspace(velocity_left[index2],velocity_left[index3],len(velocity_left[index2:index3]))


                        velocity_interp_before=velocity_left[1000-np.where(self.xsens_marker_left_array)[0][0]:index3]
                        timestamps=np.flip(self.timestamp_array[self.ids.index('00B44384'),1000-index3:np.where(self.xsens_marker_left_array)[0][0]])
                        timestamps=timestamps-timestamps[0]
                        timestamp_increments=np.round((np.diff(timestamps)/10))
                        timestamp_interpolate=np.array([0]+list(np.cumsum(timestamp_increments)))
                        velocity_interp=np.interp(np.arange(0,len(timestamp_interpolate)),timestamp_interpolate,velocity_interp_before)


                        self.left_stride_x = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0]), np.flip(self.timestamp_array[self.ids.index('00B44384'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]),[1000-np.where(self.xsens_marker_left_array)[0][0],index3],[index1,index2,index3],True)
                        self.left_stride_y = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,1]), np.flip(self.timestamp_array[self.ids.index('00B44384'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]),[1000-np.where(self.xsens_marker_left_array)[0][0],index3],[index1,index2,index3],True)
                        self.left_stride=np.sqrt(self.left_stride_x**2+self.left_stride_y**2)
                        # self.left_stride=np.sum(velocity_interp)/100
                        print('Left stride:',self.left_stride)
                        self.left_trajectory+=self.left_stride
                        
                    if len(np.where(self.xsens_marker_right_array)[0])>=3:
                        left_gyro=self.xsens_gyro_data_array[self.ids.index('00B44396'),:,2]
                        index1=20
                        index2=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1]])+np.where(self.xsens_marker_right_array)[0][0]
                        index3=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][1]:np.where(self.xsens_marker_right_array)[0][2]])+np.where(self.xsens_marker_right_array)[0][1]
                        velocity_right=np.cumsum(scipy.signal.filtfilt(self.b_filter,self.a_filter,np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0])))/100
                        index1=1000-index3
                        index2=1000-index2
                        index3=20
                        while np.std(np.sqrt(self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,0]**2+self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,1]**2+self.xsens_acceleration_data_array[self.ids.index('00B44389'),index3-20:index3+20,2]**2))<0.1:
                            index3+=1
                        index3=1000-index3
                        velocity_right[index1:index2]=velocity_right[index1:index2]-np.linspace(velocity_right[index1],velocity_right[index2],len(velocity_right[index1:index2]))
                        velocity_right[index2:index3]=velocity_right[index2:index3]-np.linspace(velocity_right[index2],velocity_right[index3],len(velocity_right[index2:index3]))
                        

                        velocity_interp_before=velocity_right[1000-np.where(self.xsens_marker_right_array)[0][0]:index3]
                        timestamps=np.flip(self.timestamp_array[self.ids.index('00B44384'),1000-index3:np.where(self.xsens_marker_right_array)[0][0]])
                        timestamps=timestamps-timestamps[0]
                        timestamp_increments=np.round((np.diff(timestamps)/10))
                        timestamp_interpolate=np.array([0]+list(np.cumsum(timestamp_increments)))
                        velocity_interp=np.interp(np.arange(0,len(timestamp_interpolate)),timestamp_interpolate,velocity_interp_before)


                        self.right_stride_x = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0]), np.flip(self.timestamp_array[self.ids.index('00B44389'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]),[1000-np.where(self.xsens_marker_right_array)[0][0],index3],[index1,index2,index3],True)
                        self.right_stride_y = calculate_stride(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,1]), np.flip(self.timestamp_array[self.ids.index('00B44389'),:]), np.flip(self.timestamp_array[self.ids.index('00B4437D'),:]), np.flip(self.timestamp_array[self.ids.index('00B44396'),:]),[1000-np.where(self.xsens_marker_right_array)[0][0],index3],[index1,index2,index3],True)
                        self.right_stride=np.sqrt(self.right_stride_x**2+self.right_stride_y**2)
                        # self.right_stride=np.sum(velocity_interp)/100
                        print('Right stride:',self.right_stride)
                        self.right_trajectory+=self.right_stride

                        


                    print('Number of left steps left: ', self.calculate_stride_left)
                    print('Number of left steps right: ', self.calculate_stride_right)
                    print('Trajectory left:: ', self.left_trajectory)
                    print('Trajectory right:', self.right_trajectory)
                    # if self.calculate_stride_right:    
                    #     left_gyro=self.xsens_gyro_data_array[self.ids.index('00B44396'),:,2]
                    #     left_gyro=-left_gyro
                       
                    #     if  np.sum(self.xsens_marker_right_array)>=3:
                    #         index1=1
                    #         index2=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][0]:np.where(self.xsens_marker_right_array)[0][1]])+np.where(self.xsens_marker_right_array)[0][0]
                    #         index3=np.argmax(left_gyro[np.where(self.xsens_marker_right_array)[0][1]:np.where(self.xsens_marker_right_array)[0][2]])+np.where(self.xsens_marker_right_array)[0][1]
                    #         velocity_right=np.cumsum(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44389'),:,0]))/100
                    #         index1=1000-index3
                    #         index2=1000-index2
                    #         index3=1000-1

                    #         velocity_right[index1:index2]=velocity_right[index1:index2]-np.linspace(velocity_right[index1],velocity_right[index2],len(velocity_right[index1:index2]))
                    #         velocity_right[index2:index3]=velocity_right[index2:index3]-np.linspace(velocity_right[index2],velocity_right[index3],len(velocity_right[index2:index3]))
                    #         self.right_stride=np.sum(velocity_right[1000-np.where(self.xsens_marker_right_array)[0][1]:1000-np.where(self.xsens_marker_right_array)[0][0]])/100
                    #         print('Right stride: ',self.right_stride)
                    #         self.calculate_stride_right=False
                    # if self.calculate_stride_left:
                    #         right_gyro=self.xsens_gyro_data_array[self.ids.index('00B4437D'),:,2]
                            
                    #         if  np.sum(self.xsens_marker_left_array)>=3:
                    #             index1=1
                    #             index2=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][0]:np.where(self.xsens_marker_left_array)[0][1]])+np.where(self.xsens_marker_left_array)[0][0]
                    #             index3=np.argmax(right_gyro[np.where(self.xsens_marker_left_array)[0][1]:np.where(self.xsens_marker_left_array)[0][2]])+np.where(self.xsens_marker_left_array)[0][1]
                    #             velocity_left=np.cumsum(np.flip(self.xsens_acceleration_data_array[self.ids.index('00B44384'),:,0]))/100
                    #             index1=1000-index3
                    #             index2=1000-index2
                    #             index3=1000-1

                    #             velocity_left[index1:index2]=velocity_left[index1:index2]-np.linspace(velocity_left[index1],velocity_left[index2],len(velocity_left[index1:index2]))
                    #             velocity_left[index2:index3]=velocity_left[index2:index3]-np.linspace(velocity_left[index2],velocity_left[index3],len(velocity_left[index2:index3]))
                    #             self.left_stride=np.sum(velocity_left[1000-np.where(self.xsens_marker_left_array)[0][1]:1000-np.where(self.xsens_marker_left_array)[0][0]])/100
                    #             print('Left stride:',self.left_stride)
                    #             self.calculate_stride_left=False

                    

                self.xsens_marker_left_array[1:]=self.xsens_marker_left_array[:999]
                self.xsens_marker_right_array[1:]=self.xsens_marker_right_array[:999]
                self.xsens_marker_left_array[0]=0
                self.xsens_marker_right_array[0]=0
                if self.xsens_marker_right:
                    if self.xsens_marker_right==1000:
                        self.xsens_marker_right_array[0]=1
                    else:
                        self.xsens_marker_right_array[int(self.xsens_marker_right)]=1
                if self.xsens_marker_left:
                    if self.xsens_marker_left==1000:
                        self.xsens_marker_left_array[0]=1
                    else:
                        self.xsens_marker_left_array[int(self.xsens_marker_left)]=1

                self.tracking_markers_old=self.tracking_markers*1
                if self.additional_left_stride and not self.left_stride:
                    self.left_stride=self.additional_left_stride*1
                    self.additional_left_stride=0
                if self.additional_right_stride and not self.right_stride:
                    self.right_stride=self.additional_right_stride*1
                    self.additional_right_stride=0

                self.processed_data.emit(self.xsens_orientation_data,self.oriented_acceleration_data,self.xsens_marker_right,self.xsens_marker_left,self.timestamps[0])
                
                self.xsens_orientation_data_array[:,1:1000,:]=self.xsens_orientation_data_array[:,0:999,:]
                self.xsens_orientation_data_array[:,0,:]=self.xsens_orientation_data*1
                
                self.xsens_acceleration_data_array[:,1:1000,:]=self.xsens_acceleration_data_array[:,0:999,:]
                self.xsens_acceleration_data_array[:,0,:]=self.oriented_acceleration_data*1

                self.xsens_gyro_data_array[:,1:1000,:]=self.xsens_gyro_data_array[:,0:999,:]
                self.xsens_gyro_data_array[:,0,:]=self.xsens_gyro_data[0]

                self.timestamp_array[:,1:1000]=self.timestamp_array[:,0:999]
                self.timestamp_array[:,0]=self.timestamp_array[:,1]+np.round((self.timestamps[0]-self.timestamp_array[:,1])/10)*10

                if self.reccording :
                    reccord_data=np.reshape(self.xsens_acceleration_data_array[:,0,:],(18,)).astype(str).tolist().append(self.timestamps[0].tolist())
                    acceleration_reccord_data=np.reshape(self.xsens_acceleration_data_array[:,0,:],(18,)).astype(str).tolist()
                    orientation_reccord_data=np.reshape(self.xsens_orientation_data_array[:,0,:],(18,)).astype(str).tolist()
                    gyro_reccord_data=np.reshape(self.xsens_gyro_data_array[:,0,:],(18,)).astype(str).tolist()
                    quaternion_reccord_data=np.reshape(np.array(self.xsens_quaternion_data[0]),(24,)).astype(str).tolist()
                    reccord_data=acceleration_reccord_data+gyro_reccord_data+orientation_reccord_data+quaternion_reccord_data+self.timestamps[0].tolist()+[self.xsens_marker_left,self.xsens_marker_right,self.left_stride,self.right_stride]
                    zipobj=zip(self.field_names,reccord_data)
                    self.writer.writerow(dict(zipobj))

                self.xsens_quaternion_data.pop(0)
                self.xsens_acceleration_data.pop(0)
                self.timestamps.pop(0)
                self.xsens_gyro_data.pop(0)


                

                


            self.usleep(100)

    def recieve(self,quaternion_data, acceleration_data, init_quats,timestamp,gyro):
        self.xsens_quaternion_data.append(quaternion_data)
        self.xsens_acceleration_data.append(acceleration_data)
        self.init_quats=init_quats
        self.timestamps.append(timestamp)
        self.xsens_gyro_data.append(gyro)


    def marker_state(self,indicator):
        self.xsens_marker_right=0
        self.xsens_marker_left=0
        self.tracking_markers=indicator
        self.angle_trend_left_1=0
        self.angle_trend_left_2=0
        self.angle_trend_right_1=0
        self.angle_trend_right_2=0
        self.left_marker_distance=0
        self.right_marker_distance=0
        self.previous_angle_left=0
        self.previous_angle_right=0
        self.state_left=True
        self.state_right=True
        self.left_shank_angles=np.zeros(10000)
        self.right_shank_angles=np.zeros(10000)        
    def init_csv_writer(self):
        self.filename=str(datetime.now())
        self.filename=self.filename.replace(' ','_')
        self.filename=self.filename.replace('.','_')
        self.filename=self.filename.replace(':','_')
        self.filename=self.filename.replace('-','_')
        self.file_handler=open(self.filename+'.csv','w',newline='\n')
        self.field_names=[]
        for name in self.ids:
            self.field_names.append(name+' - acc x')
            self.field_names.append(name+' - acc y')
            self.field_names.append(name+' - acc z')
        for name in self.ids:
            self.field_names.append(name+' - gyro x')
            self.field_names.append(name+' - gyro y')
            self.field_names.append(name+' - gyro z')
        for name in self.ids:
            self.field_names.append(name+' - sagital')
            self.field_names.append(name+' - frontal')
            self.field_names.append(name+' - transversal')
        for name in self.ids:
            self.field_names.append(name+' - q0')
            self.field_names.append(name+' - q1')
            self.field_names.append(name+' - q2')
            self.field_names.append(name+' - q3')
        for name in self.ids:
            self.field_names.append(name+' -timestamp')
        self.field_names.append('left leg marker')
        self.field_names.append('right leg marker')
        self.field_names.append('left leg stride length')
        self.field_names.append('right leg stride length')
        self.writer = csv.DictWriter(self.file_handler, fieldnames=self.field_names)
        self.writer.writeheader()


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1127, 873)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.sensor_list_label = QtWidgets.QLabel(self.centralwidget)
        self.sensor_list_label.setMaximumSize(QtCore.QSize(100, 50))
        self.sensor_list_label.setObjectName("sensor_list_label")
        self.verticalLayout_2.addWidget(self.sensor_list_label)
        self.sensor_list = QtWidgets.QListView(self.centralwidget)
        self.sensor_list.setMaximumSize(QtCore.QSize(200, 300))
        self.sensor_list.setObjectName("sensor_list")
        self.verticalLayout_2.addWidget(self.sensor_list)
        self.connection_log_label = QtWidgets.QLabel(self.centralwidget)
        self.connection_log_label.setMaximumSize(QtCore.QSize(100, 50))
        self.connection_log_label.setObjectName("connection_log_label")
        self.verticalLayout_2.addWidget(self.connection_log_label)
        self.connection_log_list = QtWidgets.QListView(self.centralwidget)
        self.connection_log_list.setMaximumSize(QtCore.QSize(200, 300))
        self.connection_log_list.setObjectName("connection_log_list")
        self.verticalLayout_2.addWidget(self.connection_log_list)
        self.finished_connecting_button = QtWidgets.QPushButton(self.centralwidget)
        self.finished_connecting_button.setObjectName("finished_connecting_button")
        self.verticalLayout_2.addWidget(self.finished_connecting_button)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.tabs = QtWidgets.QTabWidget(self.centralwidget)
        self.tabs.setObjectName("tabs")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.tab)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.xsesns_reset_button = QtWidgets.QPushButton(self.tab)
        self.xsesns_reset_button.setObjectName("xsesns_reset_button")
        self.verticalLayout.addWidget(self.xsesns_reset_button)
        self.xsens_reccord_button = QtWidgets.QPushButton(self.tab)
        self.xsens_reccord_button.setObjectName("xsens_reccord_button")
        self.verticalLayout.addWidget(self.xsens_reccord_button)
        self.xsens_markers_checkbox = QtWidgets.QCheckBox(self.tab)
        self.xsens_markers_checkbox.setObjectName("xsens_markers_checkbox")
        self.verticalLayout.addWidget(self.xsens_markers_checkbox)
        self.xsens_acceleration_label = QtWidgets.QLabel(self.tab)
        self.xsens_acceleration_label.setMaximumSize(QtCore.QSize(100, 40))
        self.xsens_acceleration_label.setObjectName("xsens_acceleration_label")
        self.verticalLayout.addWidget(self.xsens_acceleration_label)
        self.xsens_anterior_acceleration_checkbox = QtWidgets.QCheckBox(self.tab)
        self.xsens_anterior_acceleration_checkbox.setObjectName("xsens_anterior_acceleration_checkbox")
        self.verticalLayout.addWidget(self.xsens_anterior_acceleration_checkbox)
        self.xsens_lateral_acceleration_checkbox = QtWidgets.QCheckBox(self.tab)
        self.xsens_lateral_acceleration_checkbox.setObjectName("xsens_lateral_acceleration_checkbox")
        self.verticalLayout.addWidget(self.xsens_lateral_acceleration_checkbox)
        self.xsens_vertical_acceleration_checkbox = QtWidgets.QCheckBox(self.tab)
        self.xsens_vertical_acceleration_checkbox.setObjectName("xsens_vertical_acceleration_checkbox")
        self.verticalLayout.addWidget(self.xsens_vertical_acceleration_checkbox)
        self.xsens_angle_label = QtWidgets.QLabel(self.tab)
        self.xsens_angle_label.setMaximumSize(QtCore.QSize(100, 40))
        self.xsens_angle_label.setObjectName("xsens_angle_label")
        self.verticalLayout.addWidget(self.xsens_angle_label)
        self.xsens_sagital_plane_angle_checkbox = QtWidgets.QCheckBox(self.tab)
        self.xsens_sagital_plane_angle_checkbox.setObjectName("xsens_sagital_plane_angle_checkbox")
        self.verticalLayout.addWidget(self.xsens_sagital_plane_angle_checkbox)
        self.xsens_frontal_plane_angle_checkbox = QtWidgets.QCheckBox(self.tab)
        self.xsens_frontal_plane_angle_checkbox.setObjectName("xsens_frontal_plane_angle_checkbox")
        self.verticalLayout.addWidget(self.xsens_frontal_plane_angle_checkbox)
        self.xsens_transversal_plane_angle_checkbox = QtWidgets.QCheckBox(self.tab)
        self.xsens_transversal_plane_angle_checkbox.setObjectName("xsens_transversal_plane_angle_checkbox")
        self.verticalLayout.addWidget(self.xsens_transversal_plane_angle_checkbox)
        self.horizontalLayout_2.addLayout(self.verticalLayout)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.horizontalLayout_2.addLayout(self.gridLayout)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.stride_duration_right_list = QtWidgets.QListView(self.tab)
        self.stride_duration_right_list.setMaximumSize(QtCore.QSize(100, 200))
        self.stride_duration_right_list.setObjectName("stride_duration_right_list")
        self.gridLayout_2.addWidget(self.stride_duration_right_list, 4, 1, 1, 1)
        self.angles_right_list = QtWidgets.QListView(self.tab)
        self.angles_right_list.setMaximumSize(QtCore.QSize(100, 300))
        self.angles_right_list.setObjectName("angles_right_list")
        self.gridLayout_2.addWidget(self.angles_right_list, 2, 1, 1, 1)
        self.stride_duration_left_list = QtWidgets.QListView(self.tab)
        self.stride_duration_left_list.setMaximumSize(QtCore.QSize(100, 200))
        self.stride_duration_left_list.setObjectName("stride_duration_left_list")
        self.gridLayout_2.addWidget(self.stride_duration_left_list, 4, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.tab)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 3, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.tab)
        self.label_6.setObjectName("label_6")
        self.gridLayout_2.addWidget(self.label_6, 5, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.tab)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 1, 1, 1, 1)
        self.stride_length_right_list = QtWidgets.QListView(self.tab)
        self.stride_length_right_list.setMaximumSize(QtCore.QSize(100, 200))
        self.stride_length_right_list.setObjectName("stride_length_right_list")
        self.gridLayout_2.addWidget(self.stride_length_right_list, 6, 1, 1, 1)
        self.angles_left_list = QtWidgets.QListView(self.tab)
        self.angles_left_list.setMaximumSize(QtCore.QSize(100, 300))
        self.angles_left_list.setObjectName("angles_left_list")
        self.gridLayout_2.addWidget(self.angles_left_list, 2, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.tab)
        self.label_5.setObjectName("label_5")
        self.gridLayout_2.addWidget(self.label_5, 5, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.tab)
        self.label_4.setObjectName("label_4")
        self.gridLayout_2.addWidget(self.label_4, 3, 1, 1, 1)
        self.label = QtWidgets.QLabel(self.tab)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 1, 0, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.tab)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.gridLayout_2.addWidget(self.label_7, 0, 0, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.tab)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.gridLayout_2.addWidget(self.label_8, 0, 1, 1, 1)
        self.stride_length_left_list = QtWidgets.QListView(self.tab)
        self.stride_length_left_list.setMaximumSize(QtCore.QSize(100, 200))
        self.stride_length_left_list.setObjectName("stride_length_left_list")
        self.gridLayout_2.addWidget(self.stride_length_left_list, 6, 0, 1, 1)
        self.horizontalLayout_2.addLayout(self.gridLayout_2)
        self.tabs.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tabs.addTab(self.tab_2, "")
        self.horizontalLayout.addWidget(self.tabs)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1127, 21))
        self.menubar.setObjectName("menubar")
        self.menu = QtWidgets.QMenu(self.menubar)
        self.menu.setObjectName("menu")
        self.menuConnect_sensors = QtWidgets.QMenu(self.menu)
        self.menuConnect_sensors.setObjectName("menuConnect_sensors")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.settings_action = QtWidgets.QAction(MainWindow)
        self.settings_action.setObjectName("settings_action")
        self.close_action = QtWidgets.QAction(MainWindow)
        self.close_action.setObjectName("close_action")
        self.xsens_connect_action = QtWidgets.QAction(MainWindow)
        self.xsens_connect_action.setObjectName("xsens_connect_action")
        self.menuConnect_sensors.addAction(self.xsens_connect_action)
        self.menu.addAction(self.menuConnect_sensors.menuAction())
        self.menu.addAction(self.close_action)
        self.menubar.addAction(self.menu.menuAction())

        self.retranslateUi(MainWindow)
        self.tabs.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        
        self.add_functionality(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.sensor_list_label.setText(_translate("MainWindow", "Connected Sensors"))
        self.connection_log_label.setText(_translate("MainWindow", "Connection log"))
        self.finished_connecting_button.setText(_translate("MainWindow", "Finished connecting"))
        self.xsesns_reset_button.setText(_translate("MainWindow", "Reset"))
        self.xsens_reccord_button.setText(_translate("MainWindow", "Start reccording"))
        self.xsens_markers_checkbox.setText(_translate("MainWindow", "Calculate segmentation"))
        self.xsens_acceleration_label.setText(_translate("MainWindow", "Acceleration"))
        self.xsens_anterior_acceleration_checkbox.setText(_translate("MainWindow", "Anterior"))
        self.xsens_lateral_acceleration_checkbox.setText(_translate("MainWindow", "Lateral"))
        self.xsens_vertical_acceleration_checkbox.setText(_translate("MainWindow", "Vertical"))
        self.xsens_angle_label.setText(_translate("MainWindow", "Angle"))
        self.xsens_sagital_plane_angle_checkbox.setText(_translate("MainWindow", "Sagital"))
        self.xsens_frontal_plane_angle_checkbox.setText(_translate("MainWindow", "Frontal"))
        self.xsens_transversal_plane_angle_checkbox.setText(_translate("MainWindow", "Transversal"))
        self.label_3.setText(_translate("MainWindow", "Stride duration"))
        self.label_6.setText(_translate("MainWindow", "Stride length"))
        self.label_2.setText(_translate("MainWindow", "Sagital angles"))
        self.label_5.setText(_translate("MainWindow", "Stride length"))
        self.label_4.setText(_translate("MainWindow", "Stride duration"))
        self.label.setText(_translate("MainWindow", "Sagital angles"))
        self.label_7.setText(_translate("MainWindow", "Left leg"))
        self.label_8.setText(_translate("MainWindow", "Right leg"))
        self.tabs.setTabText(self.tabs.indexOf(self.tab), _translate("MainWindow", "Xsens"))
        self.tabs.setTabText(self.tabs.indexOf(self.tab_2), _translate("MainWindow", "/"))
        self.menu.setTitle(_translate("MainWindow", "Menu"))
        self.menuConnect_sensors.setTitle(_translate("MainWindow", "Connect sensors"))
        self.settings_action.setText(_translate("MainWindow", "Settings"))
        self.close_action.setText(_translate("MainWindow", "Close"))
        self.xsens_connect_action.setText(_translate("MainWindow", "Xsens MTw Awinda"))

    def add_functionality(self,MainWindow):
        
        self.close_action.triggered.connect(MainWindow.close)
        self.xsens_connect_action.triggered.connect(self.connect_xsens)
        self.finished_connecting_button.clicked.connect(self.finished_connecting)
        self.xsesns_reset_button.clicked.connect(self.reset_xsens)
        self.pyqtgraph_view=pg.GraphicsLayoutWidget()
        self.xsens_markers_checkbox.stateChanged.connect(self.markers_state_change)
        self.xsens_reccord_button.clicked.connect(self.reccording)

        self.gridLayout.addWidget(self.pyqtgraph_view)
        self.pyqtgraph_view.setBackground([255,255,255])
        self.p1 =self.pyqtgraph_view.addPlot(title="Left thigh",row=0,col=0)
        self.p2 = self.pyqtgraph_view.addPlot(title="Right thigh",row=0,col=1)
        self.p3 =self.pyqtgraph_view.addPlot(title="Left shank",row=1,col=0)
        self.p4 = self.pyqtgraph_view.addPlot(title="Right shank",row=1,col=1)
        self.p5 =self.pyqtgraph_view.addPlot(title="Left foot",row=2,col=0)
        self.p6 = self.pyqtgraph_view.addPlot(title="Right foot",row=2,col=1)
        self.pen_width=1
        self.color_array=['DE1E1B','DE8E1B','E5D621','21E55F','21E5C1','2192E5','4E21E5','8C21E5','C721E5','000000']
        qGraphicsGridLayout = self.pyqtgraph_view.ci.layout
        qGraphicsGridLayout.setColumnFixedWidth(0,600)
        qGraphicsGridLayout.setColumnFixedWidth(1,600)
        self.fs_indicator_box=[]
        self.fs_indicator_bar=[]
        for i in range(6):
            self.fs_indicator_box.append(pg.QtGui.QGraphicsRectItem())
            self.fs_indicator_bar.append(pg.QtGui.QGraphicsRectItem())
            self.fs_indicator_box[i].setPen(pg.mkPen(width=self.pen_width+1,color='k'))
            self.fs_indicator_bar[i].setPen(pg.mkPen(None))
            self.fs_indicator_bar[i].setBrush(pg.mkBrush(color=(0,0,0)))

        self.graph_item_paths1=[]
        for i in range(9):
            self.graph_item_paths1.append(pg.QtGui.QGraphicsPathItem())
            self.p1.addItem(self.graph_item_paths1[-1])
            self.graph_item_paths1[-1].setPen(pg.mkPen(width=self.pen_width,color=self.color_array[i]))
            self.p1.addItem(self.fs_indicator_box[0])
            self.p1.addItem(self.fs_indicator_bar[0])
        self.graph_item_paths2=[]
        for i in range(9):
            self.graph_item_paths2.append(pg.QtGui.QGraphicsPathItem())
            self.p2.addItem(self.graph_item_paths2[-1])
            self.graph_item_paths2[-1].setPen(pg.mkPen(width=self.pen_width,color=self.color_array[i]))
            self.p2.addItem(self.fs_indicator_box[1])
            self.p2.addItem(self.fs_indicator_bar[1])
        self.graph_item_paths3=[]
        for i in range(9):
            self.graph_item_paths3.append(pg.QtGui.QGraphicsPathItem())
            self.p3.addItem(self.graph_item_paths3[-1])
            self.graph_item_paths3[-1].setPen(pg.mkPen(width=self.pen_width,color=self.color_array[i]))
            self.p3.addItem(self.fs_indicator_box[2])
            self.p3.addItem(self.fs_indicator_bar[2])
        self.graph_item_paths4=[]
        for i in range(9):
            self.graph_item_paths4.append(pg.QtGui.QGraphicsPathItem())
            self.p4.addItem(self.graph_item_paths4[-1])
            self.graph_item_paths4[-1].setPen(pg.mkPen(width=self.pen_width,color=self.color_array[i]))
            self.p4.addItem(self.fs_indicator_box[3])
            self.p4.addItem(self.fs_indicator_bar[3])
        self.graph_item_paths5=[]
        for i in range(9):
            self.graph_item_paths5.append(pg.QtGui.QGraphicsPathItem())
            self.p5.addItem(self.graph_item_paths5[-1])
            self.graph_item_paths5[-1].setPen(pg.mkPen(width=self.pen_width,color=self.color_array[i]))
            self.p5.addItem(self.fs_indicator_box[4])
            self.p5.addItem(self.fs_indicator_bar[4])
        self.graph_item_paths6=[]
        for i in range(9):
            self.graph_item_paths6.append(pg.QtGui.QGraphicsPathItem())
            self.p6.addItem(self.graph_item_paths6[-1])
            self.graph_item_paths6[-1].setPen(pg.mkPen(width=self.pen_width,color=self.color_array[i]))
            self.p6.addItem(self.fs_indicator_box[5])
            self.p6.addItem(self.fs_indicator_bar[5])
        self.markers_plot1=pg.QtGui.QGraphicsPathItem()   
        self.markers_plot2=pg.QtGui.QGraphicsPathItem()
        self.markers_plot3=pg.QtGui.QGraphicsPathItem()
        self.markers_plot4=pg.QtGui.QGraphicsPathItem()
        self.markers_plot5=pg.QtGui.QGraphicsPathItem()
        self.markers_plot6=pg.QtGui.QGraphicsPathItem()


        self.markers_plot1.setPen(pg.mkPen(width=5,color='k'))
        self.markers_plot2.setPen(pg.mkPen(width=5,color='k'))
        self.markers_plot3.setPen(pg.mkPen(width=5,color='k'))
        self.markers_plot4.setPen(pg.mkPen(width=5,color='k'))
        self.markers_plot5.setPen(pg.mkPen(width=5,color='k'))
        self.markers_plot6.setPen(pg.mkPen(width=5,color='k'))

        self.p1.addItem(self.markers_plot1)
        self.p2.addItem(self.markers_plot2) 
        self.p3.addItem(self.markers_plot3) 
        self.p4.addItem(self.markers_plot4) 
        self.p5.addItem(self.markers_plot5) 
        self.p6.addItem(self.markers_plot6)  

        

        self.p1.disableAutoRange()
        self.p2.disableAutoRange()
        self.p3.disableAutoRange()
        self.p4.disableAutoRange()
        self.p5.disableAutoRange()
        self.p6.disableAutoRange()

        self.p1.setXRange(0, 10, padding=0)
        self.p1.setYRange(-80, 80, padding=0)
        self.p2.setXRange(0, 10, padding=0)
        self.p2.setYRange(-80, 80, padding=0)
        self.p3.setXRange(0, 10, padding=0)
        self.p3.setYRange(-80, 80, padding=0)
        self.p4.setXRange(0, 10, padding=0)
        self.p4.setYRange(-80, 80, padding=0)
        self.p5.setXRange(0, 10, padding=0)
        self.p5.setYRange(-90, 70, padding=0)
        self.p6.setXRange(0, 10, padding=0)
        self.p6.setYRange(-90, 70, padding=0)




        self.add_variables(MainWindow)
    def add_variables(self,MainWindow):

        self.sensors_connection_log_model=QtGui.QStandardItemModel()
        self.connection_log_list.setModel(self.sensors_connection_log_model)
        self.finished_connecting_indicator=False

        self.sensor_list_model=QtGui.QStandardItemModel()
        self.sensor_list.setModel(self.sensor_list_model)

        self.angles_left_list_model=QtGui.QStandardItemModel()
        self.angles_left_list.setModel(self.angles_left_list_model)

        self.angles_right_list_model=QtGui.QStandardItemModel()
        self.angles_right_list.setModel(self.angles_right_list_model)  

        self.stride_duration_left_list_model=QtGui.QStandardItemModel()
        self.stride_duration_left_list.setModel(self.stride_duration_left_list_model)

        self.stride_duration_right_list_model=QtGui.QStandardItemModel()
        self.stride_duration_right_list.setModel(self.stride_duration_right_list_model)

        self.stride_length_left_list_model=QtGui.QStandardItemModel()
        self.stride_length_left_list.setModel(self.stride_length_left_list_model)

        self.stride_length_right_list_model=QtGui.QStandardItemModel()
        self.stride_length_right_list.setModel(self.stride_length_right_list_model)
      
        self.angles_left_list.setAutoScroll(True)
        

        self.thread_read=pg.QtCore.QThread()
        self.thread_plot1=pg.QtCore.QThread(MainWindow)
        self.thread_plot2=pg.QtCore.QThread(MainWindow)
        self.thread_plot3=pg.QtCore.QThread(MainWindow)
        self.thread_plot4=pg.QtCore.QThread(MainWindow)
        self.thread_plot5=pg.QtCore.QThread(MainWindow)
        self.thread_plot6=pg.QtCore.QThread(MainWindow)




        plot_period=100

        self.timer_update1=pg.QtCore.QTimer()
        self.timer_update1.moveToThread(self.thread_plot1)
        self.timer_update1.setInterval(plot_period)
        self.timer_update1.timeout.connect(self.update1)
        self.thread_plot1.started.connect(self.timer_update1.start)
        self.p1.moveToThread(self.thread_plot1)

        self.timer_update2=pg.QtCore.QTimer()
        self.timer_update2.moveToThread(self.thread_plot2)
        self.timer_update2.setInterval(plot_period)
        self.timer_update2.timeout.connect(self.update2)
        self.thread_plot2.started.connect(self.timer_update2.start)
        self.p2.moveToThread(self.thread_plot2)

        self.timer_update3=pg.QtCore.QTimer()
        self.timer_update3.moveToThread(self.thread_plot3)
        self.timer_update3.setInterval(plot_period)
        self.timer_update3.timeout.connect(self.update3)
        self.thread_plot3.started.connect(self.timer_update3.start)
        self.p3.moveToThread(self.thread_plot3)

        self.timer_update4=pg.QtCore.QTimer()
        self.timer_update4.moveToThread(self.thread_plot4)
        self.timer_update4.setInterval(plot_period)
        self.timer_update4.timeout.connect(self.update4)
        self.thread_plot4.started.connect(self.timer_update4.start)
        self.p4.moveToThread(self.thread_plot4)

        self.timer_update5=pg.QtCore.QTimer()
        self.timer_update5.moveToThread(self.thread_plot5)
        self.timer_update5.setInterval(plot_period)
        self.timer_update5.timeout.connect(self.update5)
        self.thread_plot5.started.connect(self.timer_update5.start)
        self.p5.moveToThread(self.thread_plot5)

        self.timer_update6=pg.QtCore.QTimer()
        self.timer_update6.moveToThread(self.thread_plot6)
        self.timer_update6.setInterval(plot_period)
        self.timer_update6.timeout.connect(self.update6)
        self.thread_plot6.started.connect(self.timer_update6.start)
        self.p6.moveToThread(self.thread_plot6)
        
        
      
        
        self.xsens_orientation_data=np.zeros((6,1000,3))
        self.xsens_acceleration_data=np.zeros((6,1000,3))
        self.xsens_velocity_data=np.zeros((6,1000,3))
        self.init_quats=[]
        for i in range(6):
            self.init_quats.append(q.Quaternion(0,0,0,0))
        self.xsens_markers_left=np.zeros(1000)
        self.xsens_markers_right=np.zeros(1000)
        self.xsens_start_markers=False
        self.active_left=False
        self.active_right=False
        self.state_left=True
        self.state_right=True
        self.xsens_plotting_data=np.concatenate(( self.xsens_orientation_data,self.xsens_velocity_data,self.xsens_acceleration_data),axis=2)
        self.xsens_connected=False
        self.counter=0
        self.plot_update_counter=0
        self.xsens_indicators=np.zeros(10)
        self.time=np.arange(1000)/100
        self.timestamps=np.zeros((6,1000))
        self.reccording=False

    def connect_xsens(self):
        
        self.control = xda.XsControl_construct()
        self.xdaVersion = xda.XsVersion()
        xda.xdaVersion(self.xdaVersion)
        self.print_log(self.sensors_connection_log_model,"Using XDA version %s" % self.xdaVersion.toXsString())
        self.print_log(self.sensors_connection_log_model,"Scanning for devices...")
        portInfoArray =  xda.XsScanner_scanPorts()
        if len(portInfoArray)==0:
            self.print_log(self.sensors_connection_log_model,'No device detected, connection unsuccessful.')
        else:
            self.mtPort=portInfoArray[0]

            self.did = self.mtPort.deviceId()
            self.print_log(self.sensors_connection_log_model,"Found a device with:")
            self.print_log(self.sensors_connection_log_model," Device ID: %s" % self.did.toXsString())
            self.print_log(self.sensors_connection_log_model," Port name: %s" % self.mtPort.portName())
            self.print_log(self.sensors_connection_log_model,"Opening port...")
            if not self.control.openPort(self.mtPort.portName(), self.mtPort.baudrate()):
                self.print_log(self.sensors_connection_log_model,"Could not open port. Aborting.")
            else:
                self.device = self.control.device(self.did)
                self.print_log(self.sensors_connection_log_model,"Device: %s, with ID: %s opened." % (self.device.productCode(), self.device.deviceId().toXsString()))
                self.print_log(self.sensors_connection_log_model,"Putting device into configuration mode...")
                if not self.device.gotoConfig():
                    self.print_log(self.sensors_connection_log_model,"Could not put device into configuration mode. Aborting.")
                else:
                    self.print_log(self.sensors_connection_log_model,"Configuring the device...")
                    self.configArray = xda.XsOutputConfigurationArray()
                    self.configArray.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
                    self.configArray.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))
                    self.device.setUpdateRate(100)
                    self.device.enableRadio(channel=20)
                    self.finished_connecting_indicator=False
                    self.number_of_devices=0
                    self.printed_devices=set()
                    while not self.finished_connecting_indicator:
                        QtWidgets.qApp.processEvents()
                        try: 
                            self.children_devices=self.device.children()
                            new_devices=[i.deviceId().toXsString() for i in self.children_devices]
                            if set(new_devices)!=self.printed_devices:
                                self.print_log(self.sensors_connection_log_model,'Connected sensor: '+str(list(set(new_devices)-self.printed_devices)[0]))
                                self.printed_devices=set(new_devices)
                        except:
                            pass
                    if not self.device.gotoMeasurement():
                        self.print_log(self.sensors_connection_log_model,"Could not put device into measurement mode. Aborting.")
                    else:
                        self.children_devices=self.device.children()
                        # print(self.device.setDeviceBufferSize(120))
                        # print(self.device.createLogFile('test_file.mtb'))
                        # print(self.device.startRecording())
                        self.print_log(self.sensors_connection_log_model,'Number of sensors connected: '+str(len(self.children_devices)))
                        self.callbacks=[]
                        self.sensor_ids=[]
                        self.main_callback=XdaCallback(max_buffer_size=120)
                        self.device.addCallbackHandler(self.main_callback)
                        for i in range(len(self.children_devices)):
                            self.callbacks.append(XdaCallback(max_buffer_size=10)) 
                            self.children_devices[i].addCallbackHandler(self.callbacks[i])
                            self.print_log(self.sensor_list_model,'Xsens: '+self.children_devices[i].deviceId().toXsString())
                            QtWidgets.qApp.processEvents()
                            self.sensor_ids.append(self.children_devices[i].deviceId().toXsString())
                        


                        self.alternative_reader=Reader_thread(callbacks=self.callbacks,main_callback=self.main_callback,device_ids=self.sensor_ids)
                        self.processer=Processing_thread(device_ids=self.sensor_ids,angles_left_list_model=self.angles_left_list_model,angles_right_list_model=self.angles_right_list_model, stride_duration_left_list_model=self.stride_duration_left_list_model,stride_duration_right_list_model=self.stride_duration_right_list_model,stride_length_left_list_model=self.stride_length_left_list_model,stride_length_right_list_model=self.stride_length_right_list_model)
                        self.processer.processed_data.connect(self.update_values)
                        self.alternative_reader.data.connect(self.processer.recieve)
                        self.alternative_reader.start()
                        self.processer.start()
                        # self.alternative_reader.setPriority(6)
                        # self.thread_read.start()
                        # self.thread_read.setPriority(5)
                        self.thread_plot1.start()
                        self.thread_plot2.start()
                        self.thread_plot3.start()
                        self.thread_plot4.start()
                        self.thread_plot5.start()
                        self.thread_plot6.start()
                        self.start_time=time.time()

    def update_values(self,orientation_data,acceleration_data,marker_right,marker_left,timestamp):
        self.xsens_orientation_data[:,1:1000,:]=self.xsens_orientation_data[:,0:999,:]
        self.xsens_orientation_data[:,0,:]=orientation_data

        self.xsens_acceleration_data[:,1:1000,:]=self.xsens_acceleration_data[:,0:999,:]
        self.xsens_acceleration_data[:,0,:]=acceleration_data
        self.xsens_plotting_data=np.concatenate(( self.xsens_orientation_data,self.xsens_acceleration_data),axis=2)
        self.xsens_markers_right[1:]=self.xsens_markers_right[:-1]
        self.xsens_markers_right[0]=0
        self.xsens_markers_left[1:]=self.xsens_markers_left[:-1]
        self.xsens_markers_left[0]=0
        if marker_right:
            if marker_right==1000:
                self.xsens_markers_right[0]=1
            else:
                self.xsens_markers_right[int(marker_right)]=1
        if marker_left:
            if marker_left==1000:
                self.xsens_markers_left[0]=1
            else:
                self.xsens_markers_left[int(marker_left)]=1
        
        self.timestamps[:,1:]=self.timestamps[:,:-1]
        self.timestamps[:,0]=timestamp
        self.counter+=1
        # left_stride=0
        # right_stride=0
        # if (self.xsens_markers_right[0:2]==2).any() and (self.xsens_markers_left[0:2]==2).any():
        #     marker_left=2
        #     marker_right=2
        # if marker_left!=0:
        #     if(len(np.where(self.xsens_markers_left!=0)[0])>1):
        #         last_marker=np.where(self.xsens_markers_left!=0)[0][1]
        #         index=self.sensor_ids.index('00B44389')
        #         velocity=np.cumsum(np.flip(self.xsens_acceleration_data[index,0:last_marker,0]))/100
        #         drift=np.linspace(velocity[0],velocity[-1],len(velocity))
        #         velocity=velocity-drift
        #         print(np.sum(velocity)/100)
        #         right_stride=np.sum(velocity)/100
        # if marker_right!=0:
        #     if(len(np.where(self.xsens_markers_right!=0)[0])>1):
        #         last_marker=np.where(self.xsens_markers_right!=0)[0][1]
        #         index=self.sensor_ids.index('00B44384')
        #         velocity=np.cumsum(np.flip(self.xsens_acceleration_data[index,0:last_marker,0]))/100
        #         drift=np.linspace(velocity[0],velocity[-1],len(velocity))
        #         velocity=velocity-drift
        #         print(np.sum(velocity)/100)
        #         left_stride=np.sum(velocity)/100
        # if self.reccording :
        #     reccord_data=np.reshape(self.xsens_acceleration_data[:,0,:],(18,)).astype(str).tolist().append(timestamp.tolist())
        #     acceleration_reccord_data=np.reshape(self.xsens_acceleration_data[:,0,:],(18,)).astype(str).tolist()
        #     orientation_reccord_data=np.reshape(self.xsens_orientation_data[:,0,:],(18,)).astype(str).tolist()
        #     reccord_data=acceleration_reccord_data+orientation_reccord_data+timestamp.tolist()+[marker_left,marker_right,left_stride,right_stride]
        #     zipobj=zip(self.field_names,reccord_data)
        #     self.writer.writerow(dict(zipobj))
    def markers_state_change(self):
        
        self.xsens_markers_right[1:]=self.xsens_markers_right[:-1]
        self.xsens_markers_left[1:]=self.xsens_markers_left[:-1]
        self.processer.marker_state(self.xsens_markers_checkbox.isChecked())
    def finished_connecting(self):
        self.finished_connecting_indicator=True
    def reset_xsens(self):
        for i in self.children_devices:
            i.resetOrientation(1)
        self.alternative_reader.clear_init_quats()
    def reccording(self):
        if self.processer.reccording:
            self.xsens_reccord_button.setText('Start reccording')
            # self.reccording=False
            self.counter=0
            self.processer.reccording=False
        else:
            self.processer.init_csv_writer()
            self.xsens_reccord_button.setText('Stop reccording')
            self.counter=0
            self.processer.reccording=True




    def print_log(self,model,string):
        item=QtGui.QStandardItem(string)
        model.appendRow(item)
        QtWidgets.qApp.processEvents()
    def connected_sensors_log(self,string):
        item=QtGui.QStandardItem(string)
        self.sensors_connection_log_model.appendRow(item)
        QtWidgets.qApp.processEvents()

    def update1(self):
        if '00B443B2' in self.sensor_ids:
            self.xsens_indicators[0]=self.xsens_sagital_plane_angle_checkbox.isChecked()
            self.xsens_indicators[1]=self.xsens_frontal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[2]=self.xsens_transversal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[3]=self.xsens_anterior_acceleration_checkbox.isChecked()
            self.xsens_indicators[4]=self.xsens_lateral_acceleration_checkbox.isChecked()
            self.xsens_indicators[5]=self.xsens_vertical_acceleration_checkbox.isChecked()
            self.xsens_indicators[6]=self.xsens_markers_checkbox.isChecked()
            index=self.sensor_ids.index('00B443B2')
            xmin=self.p1.viewRange()[0][0]
            xmax=self.p1.viewRange()[0][1]
            ymin=self.p1.viewRange()[1][0]
            ymax=self.p1.viewRange()[1][1]
            max_time=np.max(self.timestamps[index,:])
            min_time=np.min(self.timestamps[index,:]+1e20*(self.timestamps[index,:]==0))
            n_of_samples=np.sum(self.timestamps[index,:]!=0)
            percentage=n_of_samples/(max_time-min_time)*10
            scaling_factor=np.max([(percentage-0.9),0])
            scaling_factor=np.min([scaling_factor,0.1])
            scaling_factor*=10
            self.fs_indicator_bar[0].setBrush(pg.mkBrush(color=((1-scaling_factor)*255,scaling_factor*255,0)))
            self.fs_indicator_bar[0].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12*scaling_factor,(ymax-ymin)/25)            
            self.fs_indicator_box[0].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12,(ymax-ymin)/25)

            
            for i in range(6):
                if(self.xsens_indicators[i]):
                    path = pg.arrayToQPath(self.time, self.xsens_plotting_data[index,:,i])
                    self.graph_item_paths1[i].setPath(path)
                    # if self.xsens_indicators[6]:
                    plotting_y=np.repeat(self.xsens_plotting_data[index,np.where(self.xsens_markers_left)[0],i],2)
                    plotting_y[0::2]=plotting_y[0::2]-1
                    path=pg.arrayToQPath(np.repeat(np.where(self.xsens_markers_left)[0]/100,2),plotting_y,1-np.tile(np.arange(2),len(np.where(self.xsens_markers_left)[0])))
                    self.markers_plot1.setPath(path)
                else:
                    path = pg.arrayToQPath(np.zeros(0), np.zeros(0))
                    self.graph_item_paths1[i].setPath(path)
        

    def update2(self):
        if '00B443B5' in self.sensor_ids:
            self.xsens_indicators[0]=self.xsens_sagital_plane_angle_checkbox.isChecked()
            self.xsens_indicators[1]=self.xsens_frontal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[2]=self.xsens_transversal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[3]=self.xsens_anterior_acceleration_checkbox.isChecked()
            self.xsens_indicators[4]=self.xsens_lateral_acceleration_checkbox.isChecked()
            self.xsens_indicators[5]=self.xsens_vertical_acceleration_checkbox.isChecked()
            self.xsens_indicators[6]=self.xsens_markers_checkbox.isChecked()
            index=self.sensor_ids.index('00B443B5')
            max_time=np.max(self.timestamps[index,:])
            min_time=np.min(self.timestamps[index,:]+1e20*(self.timestamps[index,:]==0))
            n_of_samples=np.sum(self.timestamps[index,:]!=0)
            percentage=n_of_samples/(max_time-min_time)*10
            scaling_factor=np.max([(percentage-0.9),0])
            scaling_factor=np.min([scaling_factor,0.1])
            scaling_factor*=10
            xmin=self.p2.viewRange()[0][0]
            xmax=self.p2.viewRange()[0][1]
            ymin=self.p2.viewRange()[1][0]
            ymax=self.p2.viewRange()[1][1]
            self.fs_indicator_bar[1].setBrush(pg.mkBrush(color=((1-scaling_factor)*255,scaling_factor*255,0)))
            self.fs_indicator_bar[1].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12*scaling_factor,(ymax-ymin)/25)            
            self.fs_indicator_box[1].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12,(ymax-ymin)/25)

            for i in range(6):
                if(self.xsens_indicators[i]):
                    path = pg.arrayToQPath(self.time, self.xsens_plotting_data[index,:,i])
                    self.graph_item_paths2[i].setPath(path)
                    # if self.xsens_indicators[6]:
                    plotting_y=np.repeat(self.xsens_plotting_data[index,np.where(self.xsens_markers_right)[0],i],2)
                    plotting_y[0::2]=plotting_y[0::2]-1
                    path=pg.arrayToQPath(np.repeat(np.where(self.xsens_markers_right)[0]/100,2),plotting_y,1-np.tile(np.arange(2),len(np.where(self.xsens_markers_right)[0])))
                    self.markers_plot2.setPath(path)
                else:
                    path = pg.arrayToQPath(np.zeros(0), np.zeros(0))
                    self.graph_item_paths2[i].setPath(path)
        
    def update3(self):
        if '00B44396' in self.sensor_ids:
            self.xsens_indicators[0]=self.xsens_sagital_plane_angle_checkbox.isChecked()
            self.xsens_indicators[1]=self.xsens_frontal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[2]=self.xsens_transversal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[3]=self.xsens_anterior_acceleration_checkbox.isChecked()
            self.xsens_indicators[4]=self.xsens_lateral_acceleration_checkbox.isChecked()
            self.xsens_indicators[5]=self.xsens_vertical_acceleration_checkbox.isChecked()
            self.xsens_indicators[6]=self.xsens_markers_checkbox.isChecked()
            index=self.sensor_ids.index('00B44396')
            max_time=np.max(self.timestamps[index,:])
            min_time=np.min(self.timestamps[index,:]+1e20*(self.timestamps[index,:]==0))
            n_of_samples=np.sum(self.timestamps[index,:]!=0)
            percentage=n_of_samples/(max_time-min_time)*10
            scaling_factor=np.max([(percentage-0.9),0])
            scaling_factor=np.min([scaling_factor,0.1])
            scaling_factor*=10
            xmin=self.p3.viewRange()[0][0]
            xmax=self.p3.viewRange()[0][1]
            ymin=self.p3.viewRange()[1][0]
            ymax=self.p3.viewRange()[1][1]
            self.fs_indicator_bar[2].setBrush(pg.mkBrush(color=((1-scaling_factor)*255,scaling_factor*255,0)))
            self.fs_indicator_bar[2].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12*scaling_factor,(ymax-ymin)/25)            
            self.fs_indicator_box[2].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12,(ymax-ymin)/25)
            for i in range(6):
                if(self.xsens_indicators[i]):
                    path = pg.arrayToQPath(self.time, self.xsens_plotting_data[index,:,i])
                    self.graph_item_paths3[i].setPath(path)
                else:
                    path = pg.arrayToQPath(np.zeros(0), np.zeros(0))
                    self.graph_item_paths3[i].setPath(path)
            for i in range(6):
                if(self.xsens_indicators[i]):
                    path = pg.arrayToQPath(self.time, self.xsens_plotting_data[index,:,i])
                    self.graph_item_paths3[i].setPath(path)
                    # if self.xsens_indicators[6]:
                    plotting_y=np.repeat(self.xsens_plotting_data[index,np.where(self.xsens_markers_left)[0],i],2)
                    plotting_y[0::2]=plotting_y[0::2]-1
                    path=pg.arrayToQPath(np.repeat(np.where(self.xsens_markers_left)[0]/100,2),plotting_y,1-np.tile(np.arange(2),len(np.where(self.xsens_markers_left)[0])))
                    self.markers_plot3.setPath(path)
                else:
                    path = pg.arrayToQPath(np.zeros(0), np.zeros(0))
                    self.graph_item_paths3[i].setPath(path)
          

    def update4(self):
        if '00B4437D' in self.sensor_ids:
            self.xsens_indicators[0]=self.xsens_sagital_plane_angle_checkbox.isChecked()
            self.xsens_indicators[1]=self.xsens_frontal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[2]=self.xsens_transversal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[3]=self.xsens_anterior_acceleration_checkbox.isChecked()
            self.xsens_indicators[4]=self.xsens_lateral_acceleration_checkbox.isChecked()
            self.xsens_indicators[5]=self.xsens_vertical_acceleration_checkbox.isChecked()
            self.xsens_indicators[6]=self.xsens_markers_checkbox.isChecked()
            index=self.sensor_ids.index('00B4437D')
            max_time=np.max(self.timestamps[index,:])
            min_time=np.min(self.timestamps[index,:]+1e20*(self.timestamps[index,:]==0))
            n_of_samples=np.sum(self.timestamps[index,:]!=0)
            percentage=n_of_samples/(max_time-min_time)*10
            scaling_factor=np.max([(percentage-0.9),0])
            scaling_factor=np.min([scaling_factor,0.1])
            scaling_factor*=10
            xmin=self.p4.viewRange()[0][0]
            xmax=self.p4.viewRange()[0][1]
            ymin=self.p4.viewRange()[1][0]
            ymax=self.p4.viewRange()[1][1]
            self.fs_indicator_bar[3].setBrush(pg.mkBrush(color=((1-scaling_factor)*255,scaling_factor*255,0)))
            self.fs_indicator_bar[3].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12*scaling_factor,(ymax-ymin)/25)            
            self.fs_indicator_box[3].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12,(ymax-ymin)/25)
            
            for i in range(6):
                if(self.xsens_indicators[i]):
                    path = pg.arrayToQPath(self.time, self.xsens_plotting_data[index,:,i])
                    self.graph_item_paths4[i].setPath(path)
                    # if self.xsens_indicators[6]:
                    plotting_y=np.repeat(self.xsens_plotting_data[index,np.where(self.xsens_markers_right)[0],i],2)
                    plotting_y[0::2]=plotting_y[0::2]-1
                    path=pg.arrayToQPath(np.repeat(np.where(self.xsens_markers_right)[0]/100,2),plotting_y,1-np.tile(np.arange(2),len(np.where(self.xsens_markers_right)[0])))
                    self.markers_plot4.setPath(path)
                else:
                    path = pg.arrayToQPath(np.zeros(0), np.zeros(0))
                    self.graph_item_paths4[i].setPath(path)
       
    def update5(self):
        if '00B44384' in self.sensor_ids:
            self.xsens_indicators[0]=self.xsens_sagital_plane_angle_checkbox.isChecked()
            self.xsens_indicators[1]=self.xsens_frontal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[2]=self.xsens_transversal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[3]=self.xsens_anterior_acceleration_checkbox.isChecked()
            self.xsens_indicators[4]=self.xsens_lateral_acceleration_checkbox.isChecked()
            self.xsens_indicators[5]=self.xsens_vertical_acceleration_checkbox.isChecked()
            self.xsens_indicators[6]=self.xsens_markers_checkbox.isChecked()
            index=self.sensor_ids.index('00B44384')
            max_time=np.max(self.timestamps[index,:])
            min_time=np.min(self.timestamps[index,:]+1e20*(self.timestamps[index,:]==0))
            n_of_samples=np.sum(self.timestamps[index,:]!=0)
            percentage=n_of_samples/(max_time-min_time)*10
            scaling_factor=np.max([(percentage-0.9),0])
            scaling_factor=np.min([scaling_factor,0.1])
            scaling_factor*=10
            xmin=self.p5.viewRange()[0][0]
            xmax=self.p5.viewRange()[0][1]
            ymin=self.p5.viewRange()[1][0]
            ymax=self.p5.viewRange()[1][1]
            self.fs_indicator_bar[4].setBrush(pg.mkBrush(color=((1-scaling_factor)*255,scaling_factor*255,0)))
            self.fs_indicator_bar[4].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12*scaling_factor,(ymax-ymin)/25)            
            self.fs_indicator_box[4].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12,(ymax-ymin)/25)
            for i in range(6):
                if(self.xsens_indicators[i]):
                    path = pg.arrayToQPath(self.time, self.xsens_plotting_data[index,:,i])
                    self.graph_item_paths5[i].setPath(path)
                else:
                    path = pg.arrayToQPath(np.zeros(0), np.zeros(0))
                    self.graph_item_paths5[i].setPath(path)
            for i in range(6):
                if(self.xsens_indicators[i]):
                    path = pg.arrayToQPath(self.time, self.xsens_plotting_data[index,:,i])
                    self.graph_item_paths5[i].setPath(path)
                    # if self.xsens_indicators[6]:
                    plotting_y=np.repeat(self.xsens_plotting_data[index,np.where(self.xsens_markers_left)[0],i],2)
                    plotting_y[0::2]=plotting_y[0::2]-1
                    path=pg.arrayToQPath(np.repeat(np.where(self.xsens_markers_left)[0]/100,2),plotting_y,1-np.tile(np.arange(2),len(np.where(self.xsens_markers_left)[0])))
                    self.markers_plot5.setPath(path)
                else:
                    path = pg.arrayToQPath(np.zeros(0), np.zeros(0))
                    self.graph_item_paths5[i].setPath(path)      

    def update6(self):
        if '00B44389' in self.sensor_ids:
            self.xsens_indicators[0]=self.xsens_sagital_plane_angle_checkbox.isChecked()
            self.xsens_indicators[1]=self.xsens_frontal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[2]=self.xsens_transversal_plane_angle_checkbox.isChecked()
            self.xsens_indicators[3]=self.xsens_anterior_acceleration_checkbox.isChecked()
            self.xsens_indicators[4]=self.xsens_lateral_acceleration_checkbox.isChecked()
            self.xsens_indicators[5]=self.xsens_vertical_acceleration_checkbox.isChecked()
            self.xsens_indicators[6]=self.xsens_markers_checkbox.isChecked()
            index=self.sensor_ids.index('00B44389')
            max_time=np.max(self.timestamps[index,:])
            min_time=np.min(self.timestamps[index,:]+1e20*(self.timestamps[index,:]==0))
            n_of_samples=np.sum(self.timestamps[index,:]!=0)
            percentage=n_of_samples/(max_time-min_time)*10
            scaling_factor=np.max([(percentage-0.9),0])
            scaling_factor=np.min([scaling_factor,0.1])
            scaling_factor*=10
            xmin=self.p6.viewRange()[0][0]
            xmax=self.p6.viewRange()[0][1]
            ymin=self.p6.viewRange()[1][0]
            ymax=self.p6.viewRange()[1][1]
            self.fs_indicator_bar[5].setBrush(pg.mkBrush(color=((1-scaling_factor)*255,scaling_factor*255,0)))
            self.fs_indicator_bar[5].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12*scaling_factor,(ymax-ymin)/25)            
            self.fs_indicator_box[5].setRect(xmax-(xmax-xmin)/10*2,ymax-(ymax-ymin)/25*2,(xmax-xmin)/12,(ymax-ymin)/25)
            for i in range(6):
                if(self.xsens_indicators[i]):
                    path = pg.arrayToQPath(self.time, self.xsens_plotting_data[index,:,i])
                    self.graph_item_paths6[i].setPath(path)
                    # if self.xsens_indicators[6]:
                    plotting_y=np.repeat(self.xsens_plotting_data[index,np.where(self.xsens_markers_right)[0],i],2)
                    plotting_y[0::2]=plotting_y[0::2]-1
                    path=pg.arrayToQPath(np.repeat(np.where(self.xsens_markers_right)[0]/100,2),plotting_y,1-np.tile(np.arange(2),len(np.where(self.xsens_markers_right)[0])))
                    self.markers_plot6.setPath(path)
                else:
                    path = pg.arrayToQPath(np.zeros(0), np.zeros(0))
                    self.graph_item_paths6[i].setPath(path)
        



    def __del__(self):
        try:
            
            self.thread_read.quit()
            self.thread_plot1.quit()
            self.thread_plot2.quit()
            self.thread_plot3.quit()
            self.thread_plot4.quit()
            self.thread_plot5.quit()
            self.thread_plot6.quit()
            self.alternative_reader.quit()
            # print(time.time()-self.start_time)
            self.device.disableRadio()
            self.device.clearCallbackHandlers()
            self.control.closePort(self.mtPort.portName())
            self.control.close()
        except:
            pass


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.showFullScreen()
    app.exec_()
    del ui
    del MainWindow