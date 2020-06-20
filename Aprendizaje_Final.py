# -*- encoding: UTF-8 -*-
import time

import motion
import argparse
import numpy as np
from naoqi import ALProxy
from visual import *
import vpykinect
import sys

PORT = 9559
#robotIP =  "192.168.10.162"
robotIP =  "192.168.56.1"#


def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def get_angles(KneeLeftPos, HipLeftPos, AnkleLeftPos):
    trans_a = HipLeftPos - KneeLeftPos
    trans_b = AnkleLeftPos - KneeLeftPos
    angles = np.arccos(np.sum(trans_a * trans_b, axis = 0)/(np.sqrt(np.sum(trans_a ** 2, axis = 0)) * np.sqrt(np.sum(trans_b ** 2, axis = 0))))
    return (angles) * (180/np.pi)

def angulos_2D(base,punto):
    orientacion=punto-base
    if orientacion[0]<0:
        angulo_x = np.arctan(orientacion[2] / orientacion[0])-np.pi
        if orientacion[2]>0:
            angulo_x = np.arctan(orientacion[2] / orientacion[0]) + np.pi
    else:
        angulo_x=np.arctan(orientacion[2]/orientacion[0])
    return (angulo_x) * (180/np.pi)

def angulos_2D_zy1(base,punto):
    orientacion=punto-base
    if orientacion[2]>0:
        angulo_x = np.arctan(orientacion[1] / orientacion[2])-np.pi
        if orientacion[1]<0:
            angulo_x = np.arctan(orientacion[1] / orientacion[2]) + np.pi
    else:
        angulo_x=np.arctan(orientacion[1]/orientacion[2])
    return (angulo_x) * (180/np.pi)

def rotacion(base, punto):
    orientacion = punto - base
    if orientacion[2] > 0:
        angulo_x = -(np.arctan(orientacion[1] / orientacion[2])) + np.pi
        if orientacion[1] < 0:
            angulo_x = -(np.arctan(orientacion[1] / orientacion[2])) - np.pi
    else:
        angulo_x = -(np.arctan(orientacion[1] / orientacion[2]))
    return (angulo_x) * (180 / np.pi)

def angulos_2D_zy2(base,punto):
    orientacion=punto-base
    if orientacion[0]>0:
        angulo_x = np.arctan(orientacion[1] / orientacion[0])-np.pi
        if orientacion[1]<0:
            angulo_x = np.arctan(orientacion[1] / orientacion[0]) + np.pi
    else:
        angulo_x=np.arctan(orientacion[1]/orientacion[0])
    return (angulo_x) * (-180/np.pi)

def angulos_2D_zy(base,punto):
    orientacion=punto-base
    angulo_x=np.arctan(orientacion[2]/orientacion[1])
    return (angulo_x) * (180/np.pi)

def angulos_2D_yx(base,punto):
    orientacion=punto-base
    angulo_x=np.arctan(orientacion[0]/orientacion[1])
    return (angulo_x) * (180/np.pi)



def angulo_lateral(base,punto,trans_b):
    trans_a=punto-base
    if trans_a[0]>0:
        angulo_x = -np.arccos(np.sum(trans_a * trans_b, axis=0) / (np.sqrt(np.sum(trans_a ** 2, axis=0)) * np.sqrt(np.sum(trans_b ** 2, axis=0))))
    else:
        angulo_x = np.arccos(np.sum(trans_a * trans_b, axis=0) / (np.sqrt(np.sum(trans_a ** 2, axis=0)) * np.sqrt(np.sum(trans_b ** 2, axis=0))))
    return (angulo_x) * (180/np.pi)


def angulo_lateral2(base,punto,trans_b):
    trans_a=punto-base
    angulo_x = np.arccos(np.sum(trans_a * trans_b, axis=0) / (np.sqrt(np.sum(trans_a ** 2, axis=0)) * np.sqrt(np.sum(trans_b ** 2, axis=0))))
    return (angulo_x) * (180/np.pi)

def Gestos(Origen, Punto_1, unitario):
    A = Punto_1 - Origen
    B = unitario
    angulo = np.arccos(np.sum(A * B, axis=0) / (np.sqrt(np.sum(A ** 2, axis=0)) * np.sqrt(np.sum(B ** 2, axis=0))))
    return (angulo) * (180 / np.pi)

def main(robotIP, PORT):
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    motionProxy.setStiffnesses("Body", 1.0)

    # Wake up robot
    #motionProxy.wakeUp()
    # Send robot to Pose Init
    # StiffnessOn(motionProxy)
    # postureProxy.goToPosture("StandInit", 0.5)

    while True:
        rate(30)
        skeleton.frame.visible = skeleton.update()
        if skeleton.frame.visible:

            # Puntos del Kinect
            Centro_Cadera = skeleton.joints[0]
            Espina = skeleton.joints[1]
            Hombro_C = skeleton.joints[2]
            Cabeza = skeleton.joints[3]
            Hombro_I = skeleton.joints[4]
            Codo_I = skeleton.joints[5]
            Muneca_I = skeleton.joints[6]
            Mano_I = skeleton.joints[7]
            Hombro_D = skeleton.joints[8]
            Codo_D = skeleton.joints[9]
            Muneca_D = skeleton.joints[10]
            Mano_D = skeleton.joints[11]
            Cadera_I = skeleton.joints[12]
            Rodilla_I = skeleton.joints[13]
            Tobillo_I = skeleton.joints[14]
            Pie_I = skeleton.joints[15]
            Cadera_D = skeleton.joints[16]
            Rodilla_D = skeleton.joints[17]
            Tobillo_D = skeleton.joints[18]
            Pie_D = skeleton.joints[19]
            #Cuenta = skeleton.joints[20]

            #Vectorizacion de puntos

            Vector_Hombro_D = np.array([Hombro_D.x, Hombro_D.y, Hombro_D.z])
            Vector_Codo_D = np.array([Codo_D.x, Codo_D.y, Codo_D.z])
            Vector_Muneca_D = np.array([Muneca_D.x, Muneca_D.y, Muneca_D.z])

            Vector_Hombro_I = np.array([Hombro_I.x, Hombro_I.y, Hombro_I.z])
            Vector_Codo_I = np.array([Codo_I.x, Codo_I.y, Codo_I.z])
            Vector_Muneca_I = np.array([Muneca_I.x, Muneca_I.y, Muneca_I.z])

            Vector_Cadera_D = np.array([Cadera_D.x, Cadera_D.y, Cadera_D.z])
            Vector_Rodilla_D = np.array([Rodilla_D.x, Rodilla_D.y, Rodilla_D.z])
            Vector_Tobillo_D = np.array([Tobillo_D.x, Tobillo_D.y, Tobillo_D.z])

            Vector_Cadera_I = np.array([Cadera_I.x, Cadera_I.y, Cadera_I.z])
            Vector_Rodilla_I = np.array([Rodilla_I.x, Rodilla_I.y, Rodilla_I.z])
            Vector_Tobillo_I = np.array([Tobillo_I.x, Tobillo_I.y, Tobillo_I.z])

            Vec_U_X=np.array([1,0,0])
            Vec_U_NegX = np.array([-1,0,0])
            #///////////////////////////////////////////////////////////////////////
            Vector_Hombro_DF = np.array([Hombro_D.x + 0.03, Hombro_D.y, Hombro_D.z])
            sombra_x = Vector_Codo_D - Vector_Hombro_DF
            sombra_x[0] = 0


            sombra_z=Vector_Codo_D - Vector_Hombro_DF
            sombra_z[2] = 0


            Hombro_V2 = angulos_2D_zy1(np.array(Vector_Hombro_D), np.array(Vector_Codo_D))* (np.pi / 180)
            if Hombro_V2 > 119* (np.pi / 180):
                Hombro_V2 = 119* (np.pi / 180)
            if Hombro_V2 < -119* (np.pi / 180):
                Hombro_V2 = -119* (np.pi / 180)

            HombroCodo_V2 = angulo_lateral(np.array(Vector_Hombro_DF), np.array(Vector_Codo_D), np.array(sombra_x))* (np.pi / 180)
            if HombroCodo_V2 < -76* (np.pi / 180):
                HombroCodo_V2 = -76* (np.pi / 180)
            if HombroCodo_V2 > 18* (np.pi / 180):
                HombroCodo_V2 = 18* (np.pi / 180)

            #///////////////////////////////////////////////////////////////////////

            Angulo_codo_D = get_angles(np.array(Vector_Codo_D), np.array(Vector_Hombro_D),
                              np.array(Vector_Muneca_D))

            Angulo_HCC_D = get_angles(np.array(Vector_Hombro_D), np.array(Vector_Codo_D),
                                np.array(Vector_Cadera_D))

            Angulo_hombrozx_D = angulos_2D(np.array(Vector_Hombro_D), np.array(Vector_Codo_D))

            Rota_codo_D = angulos_2D_zy2(np.array(Vector_Codo_D), np.array(Vector_Muneca_D))* (np.pi / 180)
            #print (Codo_D.z)
            # if Hombro_V2 > 0 * (np.pi / 180) and (Vector_Muneca_D - Vector_Codo_D)[1] < 0:
            #     Rota_codo_D = 0

            Giro_D = Gestos(Vector_Hombro_DF, Vector_Codo_D, Vec_U_X)


            complemento = angulo_lateral2(np.array(Vector_Hombro_DF), np.array(Vector_Codo_D), np.array(sombra_z))
            if complemento < 20:
                Rota_codo_D = rotacion(np.array(Vector_Codo_D), np.array(Vector_Muneca_D)) * (np.pi / 180)+Hombro_V2

            if Giro_D < 20:
                Hombro_V2 = 0
                Rota_codo_D = rotacion(np.array(Vector_Codo_D), np.array(Vector_Muneca_D)) * (np.pi / 180)
            # print("z=%.1f Y = %.1f x= %.1f" % (HombroCodo_V2, Hombro_V2, Angulo_codo_D))

            if Rota_codo_D < -119* (np.pi / 180):
                Rota_codo_D = -119* (np.pi / 180)
            if Rota_codo_D > 119* (np.pi / 180):
                Rota_codo_D = 119 * (np.pi / 180)
            #z_2D = (((38.0 / 45.0) * hombro_2) + 76.0) * (np.pi / 180)
            z_2D = (((47.0 / 54.0) * Angulo_hombrozx_D) + 76.0) * (np.pi / 180)
            if z_2D < -18* (np.pi / 180):
                z_2D = -18* (np.pi / 180)
            if z_2D > 1.32:
                z_2D = 1.32


            # z =(((44.0 / 75.0)*codo)-(528.0 / 5.0))* (np.pi/180)
            z = (((-44.0 / 45.0) * Angulo_codo_D) + 176) * (np.pi / 180)
            if z > 1.53:
                z = 1.53
            # print codo

            w = (((-6.0 / 5.0) * Angulo_HCC_D) + 114.0) * (np.pi / 180)
            ##        w =(((73.0 / 80.0)*Hombro)-(73.0 / 4.0))* (np.pi/180)
            ##        if w>1.2:
            ##            w=1.2


####modificacion
            # if z < 30*(np.pi / 180) :#and (Vector_Muneca_D-Vector_Codo_D)[1]<0:
            #     Rota_codo_D=0

    # ///////////////////////////////////////////////////////////////////////
            Vector_Hombro_IF = np.array([Hombro_I.x - 0.03, Hombro_I.y, Hombro_I.z])
            sombra_xI = Vector_Codo_I - Vector_Hombro_IF
            sombra_xI[0] = 0

            Hombro_V2I = angulos_2D_zy1(np.array(Vector_Hombro_I), np.array(Vector_Codo_I)) * (np.pi / 180)
            if Hombro_V2I > 119 * (np.pi / 180):
                Hombro_V2I = 119 * (np.pi / 180)
            if Hombro_V2I < -119 * (np.pi / 180):
                Hombro_V2I = -119 * (np.pi / 180)

            HombroCodo_V2I = angulo_lateral(np.array(Vector_Hombro_IF), np.array(Vector_Codo_I), np.array(sombra_xI)) * (
            np.pi / 180)
            if HombroCodo_V2I > 76 * (np.pi / 180):
                HombroCodo_V2I = 76 * (np.pi / 180)
            if HombroCodo_V2I < -18 * (np.pi / 180):
                HombroCodo_V2I = -18 * (np.pi / 180)
        # ///////////////////////////////////////////////////////////////////////

            Angulo_codo_I = get_angles(np.array(Vector_Codo_I), np.array(Vector_Hombro_I),
                                np.array(Vector_Muneca_I))

            Angulo_HCC_I = get_angles(np.array(Vector_Hombro_I), np.array(Vector_Codo_I),
                                  np.array(Vector_Cadera_I))

            Angulo_hombrozx_I = angulos_2D(-np.array(Vector_Hombro_I), -np.array(Vector_Codo_I))

            Rota_codo_I = -angulos_2D_zy1(np.array(Vector_Codo_I), np.array(Vector_Muneca_I)) * (np.pi / 180)
            if Rota_codo_I > 119 * (np.pi / 180):
                Rota_codo_I = 119 * (np.pi / 180)
            if Rota_codo_I < -119 * (np.pi / 180):
                Rota_codo_I = -119 * (np.pi / 180)
           # print(Hombro_V2I*(180/np.pi))
            # print ("codo=%.1f \n hombro = %.1f \n x = %.1f"%(codo_2,Hombro_2,hombro_2_2))

            #z_2D_2 = (((38.0 / 45.0) * hombro_2_2) - 76.0) * (np.pi / 180)
            z_2D_2 = (((47.0 / 54.0) * Angulo_hombrozx_I) - 76.0) * (np.pi / 180)
            if z_2D_2 > 18* (np.pi / 180):
                z_2D_2 = 18* (np.pi / 180)
            if z_2D_2 < -1.32:
                z_2D_2 = -1.32

                # print ("nao= %.1f"%z_2D_2)
            # print ("x= %.1f"%hombro_2_2)

            # z =(((44.0 / 75.0)*codo)-(528.0 / 5.0))* (np.pi/180)
            z_2 = (((44.0 / 45.0) * Angulo_codo_I) - 176.0) * (np.pi / 180)
            if z_2 < -1.53:
                z_2 = -1.53
            # print codo

            w_2 = (((-6.0 / 5.0) * Angulo_HCC_I) + 114.0) * (np.pi / 180)


            Angulo_RCT_I = get_angles(Vector_Rodilla_I,Vector_Cadera_I,Vector_Tobillo_I)
            Angulo_caderayz_I = angulos_2D_zy(Vector_Cadera_I, Vector_Rodilla_I)
            Angulo_caderayx_I = angulos_2D_yx(Vector_Cadera_I, Vector_Rodilla_I)


            rod_D = (((-25.0 / 26.0) * Angulo_RCT_I) + 2185.0 / 13.0) * (np.pi / 180)  # 50=120  y 180=-5
            mus_frente = (((-23.0 / 24.0) * Angulo_caderayz_I) - 7.0 / 4.0) * (np.pi / 180)  # -30=27 y 90=-88
            mus_lateral = Angulo_caderayx_I * (np.pi / 180)  # -21 a 45

            if rod_D > 2.09:  # 120:
                rod_D = 2.09  # 120
            if rod_D < -.087:  # -5:
                rod_D = -.087  # -5

            if mus_frente > .47:  # 27:
                mus_frente = .47  # 27
            if mus_frente < -1.53:  # -88:
                mus_frente = -1.53  # -88

            if mus_lateral < -.36:  # -21:
                mus_lateral = -.36  # -21
            if mus_lateral > .78:  # 45:
                mus_lateral = .78  # 45

            Angulo_RCT_D = get_angles(Vector_Rodilla_D, Vector_Cadera_D, Vector_Tobillo_D)
            Angulo_caderayz_D = angulos_2D_zy(Vector_Cadera_D, Vector_Rodilla_D)
            Angulo_caderayx_D = angulos_2D_yx(Vector_Cadera_D, Vector_Rodilla_D)


            rod_D_2 = (((-25.0 / 26.0) * Angulo_RCT_D) + 2185.0 / 13.0) * (np.pi / 180)  # 50=120  y 180=-5
            mus_frente_2 = (((-23.0 / 24.0) * Angulo_caderayz_D) - 7.0 / 4.0) * (np.pi / 180)  # -30=27 y 90=-88
            mus_lateral_2 = Angulo_caderayx_D * (np.pi / 180)  # -21 a 45

            if rod_D_2 > 2.09:  # 120:
                rod_D_2 = 2.09  # 120
            if rod_D_2 < -.087:  # -5:
                rod_D_2 = -.087  # -5

            if mus_frente_2 > .47:  # 27:
                mus_frente_2 = .47  # 27
            if mus_frente_2 < -1.53:  # -88:
                mus_frente_2 = -1.53  # -88

            if mus_lateral_2 > .36:  # 21:
                mus_lateral_2 = .36  # 21
            if mus_lateral_2 < -.78:  # -45:
                mus_lateral_2 = -.78  # -45

            # isEnabled = True
            # motionProxy.wbEnable(isEnabled)
            #
            # stateName = "Fixed"  # "Free"#"Plane"
            # supportLeg = "Legs"
            # # supportLeg = "Legs"
            # #motionProxy.wbFootState(stateName, supportLeg)
            # motionProxy.wbFootState(stateName,"RLeg")
            #
            # # isEnable = True
            #
            # motionProxy.wbEnableBalanceConstraint(isEnabled, supportLeg)




            # names = ["LShoulderRoll", "LShoulderPitch", "LElbowYaw", "LElbowRoll", "RShoulderRoll", "RShoulderPitch",
            #          "RElbowYaw", "RElbowRoll","LHipRoll", "LHipPitch", "LKneePitch", "RHipRoll", "RHipPitch", "RKneePitch"]
            # angles = [z_2D, w, Rota_codo_D, z, z_2D_2, w_2, Rota_codo_I, z_2,mus_lateral, mus_frente, rod_D, mus_lateral_2, mus_frente_2, rod_D_2]
            names = ["RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RElbowRoll", "LShoulderRoll", "LShoulderPitch",
                     "LElbowYaw", "LElbowRoll"]
            angles = [HombroCodo_V2, Hombro_V2, Rota_codo_D, z, HombroCodo_V2I,Hombro_V2I, 0, z_2]
            fractionMaxSpeed = 0.2
            #Centro = skeleton.joints[0]
            # print Centro.z



            try:
                motionProxy.setAngles(names, angles, fractionMaxSpeed)
            except :
                print "Fuera de rango"
                postureProxy.goToPosture("StandInit", 0.5)
                #sys.exit(0)

            if Centro_Cadera.z < 1 and Centro_Cadera.z > .1:
                postureProxy.goToPosture("StandInit", 0.5)
                sys.exit(0)
                # vpykinect._kinect.close()


        # else:
        #     postureProxy.goToPosture("StandInit", 0.5)
        #     print "maldito"



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=robotIP,
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=PORT,
                        help="Robot port number")

    args = parser.parse_args()
    # activacion kinect
    skeleton = vpykinect.Skeleton(frame(visible=False))
    skeleton.frame.visible = False
    raised = False
    main(args.ip, args.port)


