# Definicion de librerias
import time
import motion
import argparse
import numpy as np
from naoqi import ALProxy
from visual import *
import vpykinect
import sys

# Definicion de puerto e IP de NAO

PORT = 9559
robotIP =  "192.168.10.162"
#robotIP = "192.168.56.1"  #

# Declaracion de funciones

# Calcula el angulo formado por tres puntos
def Producto_Punto(Origen, Punto_1, Punto_2):
    A = Punto_1 - Origen
    B = Punto_2 - Origen
    angulo = np.arccos(np.sum(A * B, axis=0) / (np.sqrt(np.sum(A ** 2, axis=0)) * np.sqrt(np.sum(B ** 2, axis=0))))
    return (angulo) * (180 / np.pi)


# calcula el angulo formado por un vector con respecto a Z
# eliminando la dimencion en X
def angulos_2D_zy1(base, punto):
    orientacion = punto - base
    if orientacion[2] > 0:
        angulo_x = np.arctan(orientacion[1] / orientacion[2]) - np.pi
        if orientacion[1] < 0:
            angulo_x = np.arctan(orientacion[1] / orientacion[2]) + np.pi
    else:
        angulo_x = np.arctan(orientacion[1] / orientacion[2])
    return (angulo_x) * (180 / np.pi)


# calcula el angulo formado entre un vector y su sombra en ZY
def angulo_lateral(base, punto, B):
    A = punto - base
    if A[0] > 0:
        angulo_x = -np.arccos(
            np.sum(A * B, axis=0) / (np.sqrt(np.sum(A ** 2, axis=0)) * np.sqrt(np.sum(B ** 2, axis=0))))
    else:
        angulo_x = np.arccos(
            np.sum(A * B, axis=0) / (np.sqrt(np.sum(A ** 2, axis=0)) * np.sqrt(np.sum(B ** 2, axis=0))))
    return (angulo_x) * (180 / np.pi)

def angulos_2D_zy(base,punto):
    orientacion=punto-base
    angulo_x=np.arctan(orientacion[2]/orientacion[1])
    return (angulo_x) * (180/np.pi)

def angulos_2D_yx(base,punto):
    orientacion=punto-base
    angulo_x=np.arctan(orientacion[0]/orientacion[1])
    return (angulo_x) * (180/np.pi)

def Gestos(Origen, Punto_1, unitario):
    A = Punto_1 - Origen
    B = unitario
    angulo = np.arccos(np.sum(A * B, axis=0) / (np.sqrt(np.sum(A ** 2, axis=0)) * np.sqrt(np.sum(B ** 2, axis=0))))
    return (angulo) * (180 / np.pi)

def Distancia(p_1,p_2):
    vec = p_1-p_2
    Dis_E = np.sqrt(np.sum(vec ** 2, axis=0))
    return (Dis_E)

def main(robotIP, PORT):
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    tts = ALProxy("ALTextToSpeech", robotIP, PORT)
    tts.setLanguage("English")
    motionProxy.setStiffnesses("Body", 1.0)
    motionProxy.setWalkArmsEnabled(True, True)  # Enable arms control by Walk algorithm
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])  ## FOOT CONTACT PROTECTION
    postureProxy.goToPosture("StandInit", 0.3)
    tts.say("I am NAO, I am ready to imitate you")
    bol=True
    bol_I=True
    bandera=False
    bandera_I=False
    B_Camina=True
    B_Retro = True
    B_Derecha = True
    B_Izqui = True
    B_Detente = True
    B_Parado = True
    Modo_Gestos = False
    B_Gestos = True

    while True:
        rate(30)
        skeleton.frame.visible = skeleton.update()
        if skeleton.frame.visible:

            # Definicion de puntos del skeleto
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

            # Vectorizacion de puntos
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

            Vector_Cabeza = np.array([Cabeza.x, Cabeza.y, Cabeza.z])
            Vector_Mano_D = np.array([Mano_D.x, Mano_D.y, Mano_D.z])
            Vector_Mano_I = np.array([Mano_I.x, Mano_I.y, Mano_I.z])
            Vector_Centro_Cadera = np.array([Centro_Cadera.x,Centro_Cadera.y,Centro_Cadera.z])

            # Compensacion del hombro para coregir errores del sensor kinect
            Vector_Hombro_DF = np.array([Hombro_D.x + 0.03, Hombro_D.y, Hombro_D.z])
            sombra_x = Vector_Codo_D - Vector_Hombro_DF
            sombra_x[0] = 0

            Vector_Hombro_IF = np.array([Hombro_I.x - 0.03, Hombro_I.y, Hombro_I.z])
            sombra_xI = Vector_Codo_I - Vector_Hombro_IF
            sombra_xI[0] = 0

            #Vectores unitarios
            Vec_U_X = np.array([1,0,0])
            Vec_U_NegX = np.array([-1, 0, 0])
            Vec_U_NegZ = np.array([0,0,-1])
            Vec_U_NegY = np.array([0,-1,0])
            Vec_U_Y = np.array([0,1,0])

            Distancia_MC_D = Distancia(Vector_Cabeza, Vector_Mano_D)
            Distancia_MC_I = Distancia(Vector_Cabeza, Vector_Mano_I)
            Distancia_MCadera_D = Distancia(Vector_Centro_Cadera,Vector_Mano_D)
            Distancia_MCadera_I = Distancia(Vector_Centro_Cadera,Vector_Mano_I)

            #print ("D = %.3f  I = %.3f"%(Distancia_MCadera_D,Distancia_MCadera_I))
            if Distancia_MC_D <= 0.2 and Distancia_MC_I <= 0.2 and not B_Gestos:
                Modo_Gestos = True
                tts.say("Gestures mode activated")
                postureProxy.goToPosture("StandInit", 0.5)

            if Distancia_MCadera_D <= 0.2 and Distancia_MCadera_I <= 0.2 and B_Gestos:
                Modo_Gestos = False
                tts.say("Imitation mode activated")
                postureProxy.goToPosture("StandInit", 0.5)

            if Modo_Gestos:
                # Gestos
                Caminar = Gestos(Vector_Hombro_DF, Vector_Codo_D, Vec_U_NegZ)
                Retroceder = Gestos(Vector_Hombro_IF, Vector_Codo_I, Vec_U_NegZ)
                Giro_D = Gestos(Vector_Hombro_DF, Vector_Codo_D, Vec_U_X)
                Giro_I = Gestos(Vector_Hombro_IF, Vector_Codo_I, Vec_U_NegX)
                Detener_D = Gestos(Vector_Codo_D, Vector_Muneca_D, Vec_U_NegY)
                Detener_I = Gestos(Vector_Codo_I, Vector_Muneca_I, Vec_U_NegY)
                Parado_D = Gestos(Vector_Codo_D, Vector_Muneca_D, Vec_U_Y)
                Parado_I = Gestos(Vector_Codo_I, Vector_Muneca_I, Vec_U_Y)

                Angulo_codo_D = Producto_Punto(np.array(Vector_Codo_D), np.array(Vector_Hombro_D),
                                               np.array(Vector_Muneca_D))

                # Ajuste al dominio del NAO mediante regrecion lineal
                AnguloCodo_D = (((-44.0 / 45.0) * Angulo_codo_D) + 176) * (np.pi / 180)
                if AnguloCodo_D > 1.53:
                    AnguloCodo_D = 1.53

                Angulo_codo_I = Producto_Punto(np.array(Vector_Codo_I), np.array(Vector_Hombro_I),
                                               np.array(Vector_Muneca_I))

                # Ajuste al dominio del NAO mediante regrecion lineal
                AnguloCodo_I = (((44.0 / 45.0) * Angulo_codo_I) - 176.0) * (np.pi / 180)
                if AnguloCodo_I < -1.53:
                    AnguloCodo_I = -1.53

                # Verifica que el codo este recto
                if Angulo_codo_D > 150 and Angulo_codo_D < 180:
                    Codo_recto_D = True
                else:
                    Codo_recto_D = False

                if Angulo_codo_I > 150 and Angulo_codo_I < 180:
                    Codo_recto_I = True
                else:
                    Codo_recto_I = False

                # Establece la posicion de detener
                if Detener_D < 40 and Detener_I < 40 and Codo_recto_D and Codo_recto_I:
                    Detener = True
                else:
                    Detener = False

                # Automata para los gestos
                if not Detener:
                    if Caminar < 20 and Codo_recto_D and B_Camina and B_Parado:
                        #print "Camina"
                        motionProxy.post.moveToward(0.9, 0, 0)  # X,Y,theta
                        B_Camina = False
                        B_Retro = True
                        B_Derecha = True
                        B_Izqui = True
                        B_Detente = True
                    elif Retroceder < 20 and Codo_recto_I and B_Retro and B_Parado:
                        #print "Retrocede"
                        motionProxy.post.moveToward(-0.5, 0, 0)  # X,Y,theta
                        B_Retro = False
                        B_Camina = True
                        B_Derecha = True
                        B_Izqui = True
                        B_Detente = True
                    elif Giro_D < 20 and Codo_recto_D and B_Derecha and B_Parado:
                        #print "derecha"
                        motionProxy.post.moveToward(0, 0, -0.5)  # X,Y,theta
                        B_Derecha = False
                        B_Camina = True
                        B_Retro = True
                        B_Izqui = True
                        B_Detente = True
                    elif Giro_I < 20 and Codo_recto_I and B_Izqui and B_Parado:
                        #print "izquierda"
                        motionProxy.post.moveToward(0, 0, 0.5)  # X,Y,theta
                        B_Izqui = False
                        B_Camina = True
                        B_Retro = True
                        B_Derecha = True
                        B_Detente = True
                        # posicion de parado
                    elif Giro_D < 20 and Parado_D < 20 and Giro_I < 20 and Parado_I < 20 and not B_Parado:
                        #print "parado"
                        postureProxy.goToPosture("StandInit", 0.5)
                        B_Parado = True
                        B_Camina = True
                        B_Retro = True
                        B_Derecha = True
                        B_Izqui = True
                        B_Detente = True

                    # Posicion sentado
                    elif Giro_D < 40 and Detener_D < 40 and Giro_I < 40 and Detener_I < 40 and B_Parado:
                        #print "sentado"
                        postureProxy.goToPosture("Sit", 0.5)
                        B_Parado = False
                        B_Camina = True
                        B_Retro = True
                        B_Derecha = True
                        B_Izqui = True
                        B_Detente = True



                else:
                    if B_Detente and B_Parado:
                        #print "detente"
                        motionProxy.post.moveToward(0, 0, 0)  # X,Y,theta
                        B_Detente = False
                        B_Camina = True
                        B_Retro = True
                        B_Derecha = True
                        B_Izqui = True

                B_Gestos = True
            else:
                #print "imitacion"
                # Angulos Brazo Derecho
                HombroFrente_D = angulos_2D_zy1(np.array(Vector_Hombro_D), np.array(Vector_Codo_D)) * (np.pi / 180)
                # Ajuste al dominio del NAO
                if HombroFrente_D > 119 * (np.pi / 180):
                    HombroFrente_D = 119 * (np.pi / 180)
                if HombroFrente_D < -119 * (np.pi / 180):
                    HombroFrente_D = -119 * (np.pi / 180)

                HombroLateral_D = angulo_lateral(np.array(Vector_Hombro_DF), np.array(Vector_Codo_D),
                                                 np.array(sombra_x)) * (np.pi / 180)
                # Ajuste al dominio del NAO
                if HombroLateral_D < -76 * (np.pi / 180):
                    HombroLateral_D = -76 * (np.pi / 180)
                if HombroLateral_D > 18 * (np.pi / 180):
                    HombroLateral_D = 18 * (np.pi / 180)

                Angulo_codo_D = Producto_Punto(np.array(Vector_Codo_D), np.array(Vector_Hombro_D),
                                               np.array(Vector_Muneca_D))

                # Ajuste al dominio del NAO mediante regrecion lineal
                AnguloCodo_D = (((-44.0 / 45.0) * Angulo_codo_D) + 176) * (np.pi / 180)
                if AnguloCodo_D > 1.53:
                    AnguloCodo_D = 1.53

                # Angulos Brazo Izquierdo

                HombroFrente_I = angulos_2D_zy1(np.array(Vector_Hombro_I), np.array(Vector_Codo_I)) * (np.pi / 180)
                # Ajuste al dominio del NAO
                if HombroFrente_I > 119 * (np.pi / 180):
                    HombroFrente_I = 119 * (np.pi / 180)
                if HombroFrente_I < -119 * (np.pi / 180):
                    HombroFrente_I = -119 * (np.pi / 180)

                HombroLateral_I = angulo_lateral(np.array(Vector_Hombro_IF), np.array(Vector_Codo_I),
                                                 np.array(sombra_xI)) * (np.pi / 180)
                # Ajuste al dominio del NAO
                if HombroLateral_I > 76 * (np.pi / 180):
                    HombroLateral_I = 76 * (np.pi / 180)
                if HombroLateral_I < -18 * (np.pi / 180):
                    HombroLateral_I = -18 * (np.pi / 180)

                Angulo_codo_I = Producto_Punto(np.array(Vector_Codo_I), np.array(Vector_Hombro_I),
                                               np.array(Vector_Muneca_I))

                # Ajuste al dominio del NAO mediante regrecion lineal
                AnguloCodo_I = (((44.0 / 45.0) * Angulo_codo_I) - 176.0) * (np.pi / 180)
                if AnguloCodo_I < -1.53:
                    AnguloCodo_I = -1.53

                Angulo_rodilla_D = Producto_Punto(Vector_Rodilla_D, Vector_Tobillo_D, Vector_Cadera_D)
                Frente_D = angulos_2D_zy(Vector_Cadera_D, Vector_Rodilla_D)
                Lateral_D = angulos_2D_yx(Vector_Cadera_D, Vector_Rodilla_D)

                # NAO_rodilla_D = (((-25.0 / 26.0) * Angulo_rodilla_D) + 2185.0 / 13.0) * (np.pi / 180)  # 50=120  y 180=-5
                NAO_rodilla_D = (((-25.0 / 16.0) * Angulo_rodilla_D) + 276.25) * (np.pi / 180)
                # NAO_frente_D = (((-23.0 / 24.0) * Frente_D) - 7.0 / 4.0) * (np.pi / 180)  # -30=27 y 90=-88
                # NAO_frente_D = (((-1.15) * Frente_D) + 4.0) * (np.pi / 180)
                NAO_frente_D = (((-23.0 / 16.0) * Frente_D) - 7.0 / 4.0) * (np.pi / 180)
                NAO_lateral_D = (Lateral_D - 10.0) * (np.pi / 180)  # 21 a -45

                # pie_D=(((-67.0/115.0)*(NAO_frente_D*(180/np.pi)))-(5896.0/115.0))*(np.pi/180)

                if NAO_rodilla_D > 2.09:  # 120:
                    NAO_rodilla_D = 2.09  # 120
                if NAO_rodilla_D < -.087:  # -5:
                    NAO_rodilla_D = -.087  # -5

                if NAO_frente_D > .47:  # 27:
                    NAO_frente_D = .47  # 27
                if NAO_frente_D < -1.53:  # -88:
                    NAO_frente_D = -1.53  # -88

                if NAO_lateral_D > .36:  # 21:
                    NAO_lateral_D = .36  # 21
                if NAO_lateral_D < -.78:  # -45:
                    NAO_lateral_D = -.78  # -45

                # pie izquierdp
                Angulo_rodilla_I = Producto_Punto(Vector_Rodilla_I, Vector_Tobillo_I, Vector_Cadera_I)
                Frente_I = angulos_2D_zy(Vector_Cadera_I, Vector_Rodilla_I)
                Lateral_I = angulos_2D_yx(Vector_Cadera_I, Vector_Rodilla_I)

                # NAO_rodilla_D = (((-25.0 / 26.0) * Angulo_rodilla_D) + 2185.0 / 13.0) * (np.pi / 180)  # 50=120  y 180=-5
                NAO_rodilla_I = (((-25.0 / 16.0) * Angulo_rodilla_I) + 276.25) * (np.pi / 180)
                # NAO_frente_D = (((-23.0 / 24.0) * Frente_D) - 7.0 / 4.0) * (np.pi / 180)  # -30=27 y 90=-88
                # NAO_frente_I = (((-1.15) * Frente_I) + 4.0) * (np.pi / 180)
                NAO_frente_I = (((-23.0 / 16.0) * Frente_I) - 7.0 / 4.0) * (np.pi / 180)
                NAO_lateral_I = (Lateral_I + 5.0) * (np.pi / 180)  # -21 a 45

                if NAO_rodilla_I > 2.09:  # 120:
                    NAO_rodilla_I = 2.09  # 120
                if NAO_rodilla_I < -.087:  # -5:
                    NAO_rodilla_I = -.087  # -5

                if NAO_frente_I > .47:  # 27:
                    NAO_frente_I = .47  # 27
                if NAO_frente_I < -1.53:  # -88:
                    NAO_frente_I = -1.53  # -88

                if NAO_lateral_I < -.36:  # -21:
                    NAO_lateral_I = -.36  # -21
                if NAO_lateral_I > .78:  # 45:
                    NAO_lateral_I = .78  # 45

                # print Angulo_codo_D
                # ////////////////////////////////////
                altura_D = np.abs(Tobillo_D.y - Cadera_D.y)
                altura_I = np.abs(Tobillo_I.y - Cadera_I.y)
                # detecta moviminto en los pies

                if altura_D < altura_I - .06:
                    # if Codo_D.y<0 and HombroFrente_D
                    distancia = altura_I - altura_D
                    if (bol):
                        postureProxy.goToPosture("StandZero", 0.3)
                        bandera = True
                        # Activate Whole Body Balancer
                        isEnabled = True
                        motionProxy.wbEnable(isEnabled)

                        # Legs are constrained fixed
                        stateName = "Fixed"
                        supportLeg = "Legs"
                        motionProxy.wbFootState(stateName, supportLeg)

                        # Constraint Balance Motion
                        isEnable = True
                        supportLeg = "Legs"
                        motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

                        # Com go to LLeg
                        supportLeg = "LLeg"
                        duration = 2.0
                        motionProxy.wbGoToBalance(supportLeg, duration)

                        # RLeg is free
                        stateName = "Free"
                        supportLeg = "RLeg"
                        motionProxy.wbFootState(stateName, supportLeg)

                    # Definicion de las articulaciones del NAO
                    # names = ["RHipRoll", "RHipPitch", "RKneePitch"]

                    # Mapeo de los angulos optenidos del esqueleto a las articulaciones de NAO
                    # angles = [NAO_lateral_D, NAO_frente_D, NAO_rodilla_D]
                    names = ["RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RElbowRoll",
                             "LShoulderRoll", "LShoulderPitch", "LElbowYaw", "LElbowRoll",
                             "RHipRoll", "RHipPitch", "RKneePitch"]  # ,"RAnklePitch"]

                    # Mapeo de los angulos optenidos del esqueleto a las articulaciones de NAO
                    angles = [HombroLateral_D, HombroFrente_D, 0, AnguloCodo_D,
                              HombroLateral_I, HombroFrente_I, 0, AnguloCodo_I,
                              NAO_lateral_D, NAO_frente_D, NAO_rodilla_D]  # ,pie_D]

                    # Velocidad del movimiento
                    fractionMaxSpeed = 0.2
                    bol = False

                    try:
                        # Aplicacion del Mapeo
                        motionProxy.setAngles(names, angles, fractionMaxSpeed)
                    except:
                        print "Fuera de rango"
                        postureProxy.goToPosture("StandInit", 0.5)

                        # Example showing how to Enable Effector Control as an Optimization

                        # time.sleep(2)


                else:
                    if bandera:
                        effectorName = "RLeg"
                        isActive = False
                        motionProxy.wbEnableEffectorOptimization(effectorName, isActive)

                        isEnabled = False
                        motionProxy.wbEnable(isEnabled)
                        postureProxy.goToPosture("StandInit", 0.3)
                        bol = True
                        bandera = False


                        # print("pien derecho levantado %.2f" % distancia)
                if altura_I < altura_D - .06:
                    distancia = altura_D - altura_I
                    if (bol_I):
                        postureProxy.goToPosture("StandZero", 0.3)
                        bandera_I = True
                        # Activate Whole Body Balancer
                        isEnabled = True
                        motionProxy.wbEnable(isEnabled)

                        # Legs are constrained fixed
                        stateName = "Fixed"
                        supportLeg = "Legs"
                        motionProxy.wbFootState(stateName, supportLeg)

                        # Constraint Balance Motion
                        isEnable = True
                        supportLeg = "Legs"
                        motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

                        # Com go to LLeg
                        supportLeg = "RLeg"
                        duration = 2.0
                        motionProxy.wbGoToBalance(supportLeg, duration)

                        # RLeg is free
                        stateName = "Free"
                        supportLeg = "LLeg"
                        motionProxy.wbFootState(stateName, supportLeg)

                    # Definicion de las articulaciones del NAO
                    # names = ["RHipRoll", "RHipPitch", "RKneePitch"]

                    # Mapeo de los angulos optenidos del esqueleto a las articulaciones de NAO
                    # angles = [NAO_lateral_D, NAO_frente_D, NAO_rodilla_D]
                    names = ["RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RElbowRoll",
                             "LShoulderRoll", "LShoulderPitch", "LElbowYaw", "LElbowRoll",
                             "LHipRoll", "LHipPitch", "LKneePitch"]

                    # Mapeo de los angulos optenidos del esqueleto a las articulaciones de NAO
                    angles = [HombroLateral_D, HombroFrente_D, 0, AnguloCodo_D,
                              HombroLateral_I, HombroFrente_I, 0, AnguloCodo_I,
                              NAO_lateral_I, NAO_frente_I, NAO_rodilla_I]

                    # Velocidad del movimiento
                    fractionMaxSpeed = 0.2
                    bol_I = False

                    try:
                        # Aplicacion del Mapeo
                        motionProxy.setAngles(names, angles, fractionMaxSpeed)
                    except:
                        print "Fuera de rango"
                        postureProxy.goToPosture("StandInit", 0.5)

                else:
                    if bandera_I:
                        effectorName = "LLeg"
                        isActive = False
                        motionProxy.wbEnableEffectorOptimization(effectorName, isActive)

                        isEnabled = False
                        motionProxy.wbEnable(isEnabled)
                        postureProxy.goToPosture("StandInit", 0.3)
                        bol_I = True
                        bandera_I = False
                        #     #//////////////////////////////////

                # Definicion de las articulaciones del NAO
                names = ["RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RElbowRoll",
                         "LShoulderRoll", "LShoulderPitch", "LElbowYaw", "LElbowRoll"]

                # Mapeo de los angulos optenidos del esqueleto a las articulaciones de NAO
                angles = [HombroLateral_D, HombroFrente_D, 0, AnguloCodo_D,
                          HombroLateral_I, HombroFrente_I, 0, AnguloCodo_I]

                # Velocidad del movimiento
                fractionMaxSpeed = 0.2

                try:
                    # Aplicacion del Mapeo
                    motionProxy.setAngles(names, angles, fractionMaxSpeed)
                except:
                    print "Fuera de rango"
                    postureProxy.goToPosture("StandInit", 0.5)
                B_Gestos = False

            if Centro_Cadera.z < 1 and Centro_Cadera.z > .1:
                postureProxy.goToPosture("StandInit", 0.5)
                sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=robotIP,
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=PORT,
                        help="Robot port number")

    args = parser.parse_args()
    # activacion del kinect
    skeleton = vpykinect.Skeleton(frame(visible=False))
    skeleton.frame.visible = False
    raised = False
    main(args.ip, args.port)