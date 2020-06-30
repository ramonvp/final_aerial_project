#!/usr/bin/env python

import rospy, tf2_ros
import rosparam
import tf_conversions
import numpy as np
import tf.transformations as tr
from fiducial_msgs.msg import FiducialTransformArray

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

#Transformcion base_link a optical_link obtenido 

p_baselink2optical = Pose()
p_baselink2optical.position.x = 0.
p_baselink2optical.position.y = 0. 
p_baselink2optical.position.z = -0.02
p_baselink2optical.orientation.x = -0.707106781187
p_baselink2optical.orientation.y = 0.707106781187
p_baselink2optical.orientation.z = 0.
p_baselink2optical.orientation.w = 0.

#print (p_baselink2optical)

# DEFINICION DE CLASES PARA EL DICCIONARIO DE MARCADORES

class MarkerDictionary:
     # Clase para catalogar el diccionario de marcadores
    def __init__(self,numberOfMarkers=1,nameList='Default',idList=[],poseList=[]):
        self.numberOfMarkers = numberOfMarkers
        self.nameList = nameList
        self.idList = idList
        if poseList == []:
            self.mkrpose = []
        else:
            self.mkrpose = poseList

    def __str__(self): # esto solo sirve para que se vea bien al hacer print
        message = str()
        for i in range(0,len(self.mkrpose)):
            message += "ID "+str(self.idList[i])+"\tName: "+self.nameList[i]+" ->\t"+str(self.mkrpose[i])+"\n"
        return message

    def loadYAML(self, YAML_dict):
        n_trans = 3
        n_orient = 3 #rpy
        n_total = n_trans + n_orient
        matrix = YAML_dict['poseList']
        mrkdict = MarkerDictionary(YAML_dict['numberOfMarkers'],YAML_dict['nameList'],YAML_dict['idList'])
        for i in range(0,mrkdict.numberOfMarkers):
            pose_tmp = Pose()
            pose_tmp.position.x = matrix[i*n_total]
            pose_tmp.position.y = matrix[i*n_total+1]
            pose_tmp.position.z = matrix[i*n_total+2]

            #Transformacion de rpy (euler) a quaternion
            
            orient_tmp = tf_conversions.transformations.quaternion_from_euler(matrix[i*n_total+n_trans],matrix[i*n_total+n_trans+1], matrix[i*n_total+n_trans+2])
                        
            pose_tmp.orientation.x = orient_tmp[0]
            pose_tmp.orientation.y = orient_tmp[1]
            pose_tmp.orientation.z = orient_tmp[2]
            pose_tmp.orientation.w = orient_tmp[3]

            mrkdict.mkrpose.append(pose_tmp)

            #print(mrkdict)

        return mrkdict
    
    def getMarker(self,idNumber): #En mi clase busco con el idNumber busco el nombre y la pose
        idx = self.idList.index(idNumber)
        return self.nameList[idx],self.mkrpose[idx]

#############################################################################################################

# DEFINICION DE FUNCIONES

## Funciones de transformacion nuevas

def pose_to_pq(msg):
    """Convert a C{geometry_msgs/Pose} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.position.x, msg.position.y, msg.position.z])
    q = np.array([msg.orientation.x, msg.orientation.y,
                  msg.orientation.z, msg.orientation.w])
    return p, q


def pose_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/PoseStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return pose_to_pq(msg.pose)


def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q


def transform_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/TransformStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return transform_to_pq(msg.transform)


def msg_to_se3(msg):
    """Conversion from geometric ROS messages into SE(3)

    @param msg: Message to transform. Acceptable types - C{geometry_msgs/Pose}, C{geometry_msgs/PoseStamped},
    C{geometry_msgs/Transform}, or C{geometry_msgs/TransformStamped}
    @return: a 4x4 SE(3) matrix as a numpy array
    @note: Throws TypeError if we receive an incorrect type.
    """
    if isinstance(msg, Pose):
        p, q = pose_to_pq(msg)
    elif isinstance(msg, PoseStamped):
        p, q = pose_stamped_to_pq(msg)
    elif isinstance(msg, Transform):
        p, q = transform_to_pq(msg)
    elif isinstance(msg, TransformStamped):
        p, q = transform_stamped_to_pq(msg)
    else:
        raise TypeError("Invalid type for conversion to SE(3)")
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                str(q), np.linalg.norm(q)))
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    g = tr.quaternion_matrix(q)
    g[0:3, -1] = p
    return g

## fin de funciones de transformacion nuevas

def se3_to_pq(T):
    # Funcion que devuelve de la matriz homogenea (T), la translacion y la rotacion (en formato quaternion)
    #return T[0:3, -1],tr.quaternion_from_matrix(T[0:3,0:3])
    return T[0:3, -1],tr.quaternion_from_matrix(T)

def pq_to_transform(p,q):
    tmp_transform = Transform()
    tmp_transform.translation.x = p[0]
    tmp_transform.translation.y = p[1]
    tmp_transform.translation.z = p[2]
    tmp_transform.rotation.x = q[0]
    tmp_transform.rotation.y = q[1]
    tmp_transform.rotation.z = q[2]
    tmp_transform.rotation.w = q[3]
    return tmp_transform

def se3_to_transform(T):
    p,q = se3_to_pq(T)
    return pq_to_transform(p,q)


def fta_callback(msg):
     #print('Mensaje original Aruco:')
     #print(msg)
     #try:

     #Paso 1: Convertir a matriz las transformadas
        
        msg_out = FiducialTransformArray()
        msg_out.header = msg.header
        msg_out.header.frame_id = "world_estimate"

        msg_out.image_seq = msg.image_seq
    
        if len(msg.transforms) == 0 :
             print("No hay marcadores")
             rospy.loginfo("No markers detected.")
             msg_out.transforms = []               
        else:
            
            aruco = TransformStamped()
            aruco.header = msg.header
            aruco.transform = msg.transforms[0].transform
            """print('Mensaje trans_stamped:')
            print (aruco)"""
            aruco_number = msg.transforms[0].fiducial_id

            #Posicion del aruco respecto al mundo
            aruco_name, aruco_pose = diccionario.getMarker(aruco_number)
            #print(aruco_name, aruco_pose)
            world2fiducial_matrix = msg_to_se3(aruco_pose)
            #print("world2fiducial_matrix")
            #print(world2fiducial_matrix)

            #Paso 2: Transformacion del marker al optical link

            cam2fiducial_matrix =  msg_to_se3(aruco.transform)
            #print("cam2fiducial_matrix")
            #print(cam2fiducial_matrix)
            
            fiducial2cam_matrix = np.linalg.inv(cam2fiducial_matrix)
            #print("fiducial2cam_matrix")
            #print(fiducial2cam_matrix)

            #Paso 3: Camara a base link

            baselink2cam_matrix = msg_to_se3(p_baselink2optical)
            #print(cam2baselink_matrix)
            cam2baselink_matrix = np.linalg.inv(baselink2cam_matrix)
            #print("cam2baselink_matrix")
            #print(cam2baselink_matrix)

            #Paso 4: Encadenamiento de transformaciones

            world2cam_matrix = np.matmul(world2fiducial_matrix,fiducial2cam_matrix)
            #print("world2cam_matrix")
            #print(world2cam_matrix)

            world2baselink_matrix = np.matmul(world2cam_matrix,cam2baselink_matrix)
            #print("world2baselink_matrix")
            #print(world2baselink_matrix)

            world2baselink = Transform()
            world2baselink = se3_to_transform(world2baselink_matrix)
            """
            trans, quat = s3_to_pq(world2baselink_matrix)

            world2baselink.translation.x = trans[0]
            world2baselink.translation.y = trans[1]
            world2baselink.translation.z = trans[2]
            world2baselink.rotation.x = quat[0]
            world2baselink.rotation.y = quat[1]
            world2baselink.rotation.z = quat[2]
            world2baselink.rotation.w = quat[3]"""

            #print("RESULTADO!!")
            #print(world2baselink)


            #Chequeo resultado
            
            """  trans_world2baselink = tfBuffer_wreal.lookup_transform('world', 'firefly/base_link', aruco.header.stamp)
            #print("Mensaje posicion base_link respecto al mundo REAL")
            #print(trans_world2baselink)

            trans_baseLink2cam= tfBuffer.lookup_transform('world','fiducial_0', aruco.header.stamp)
            print("TF posicion desde world a fiducial")
            print(trans_baseLink2cam)
            print("Nuestro world a fiducial")
            print(se3_to_transform(world2fiducial_matrix))"""


            #trans_camera_optical = tfBuffer.lookup_transform('firefly/camera_ventral_optical_link','firefly/camera_ventral_link', aruco.header.stamp)
            #print("Mensaje posicion desde optical link al ventral link")
            #print(trans_camera_optical)
        
            #world_est = tfBuffer.transform(markerlist,'fiducial_'+str(fiducial_number))
            #pub.publish(aruco_world_tf)

            msg_out.transforms.append(world2baselink)
            #msg_out.transforms[0].transform = trans_worldest2baselink.transform
            print("msg_out: ")
            print(msg_out)

            pub.publish(msg_out)
         
            """ except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #rate.sleep()
            print (' ERROR ')
            """

if __name__=='__main__':
    rospy.init_node('node_aruco_tf')

    #Cargar diccionario de marcadores
    groundMarkers = rospy.get_param('groundMarkers')
    diccionario = MarkerDictionary()
    diccionario = diccionario.loadYAML(groundMarkers)
    print("Diccionario cargado!")
    print(diccionario)

    tfBuffer_wreal = tf2_ros.Buffer()
    listener_wreal = tf2_ros.TransformListener(tfBuffer_wreal)


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    sub = rospy.Subscriber('/fiducial_transforms',FiducialTransformArray,fta_callback)
    # El publisher y el subscriber pueden tener el mismo nombre de topic?
    # El publisher necesita un callback? Hace falta hacer algo despues de enviar un mensaje?
    pub = rospy.Publisher('/world_estimate',FiducialTransformArray,queue_size=1)
    

    rospy.spin()

