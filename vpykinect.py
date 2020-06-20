from visual import *
import pykinect
from pykinect import nui
from pykinect.nui import JointId


class Skeleton:

    #Representacion del esqueleto obtenido del kinect en VPython.
    def __init__(self, f):
        #Dibujo del esqueleto a partir de figuras geometricas
        self.frame = f
        self.joints = [sphere(frame=f, radius=0.08, color=color.yellow)
                       for i in range(20)]
        self.joints[3].radius = 0.125
        self.bones = [cylinder(frame=f, radius=0.05, color=color.yellow)
                      for bone in _bone_ids]

    def update(self):

        #Actualizacion de los puntos del esqueleto a partir del sensor de profundidad
        #Retorna verdadero si el sensor detecta movimiento del esqueleto

        updated = False
        for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
            if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:

                # Movimiento de las articulaciones.
                for joint, p in zip(self.joints, skeleton.SkeletonPositions):
                    joint.pos = (p.x, p.y, p.z)

                # Movimiento de los huesos.
                for bone, bone_id in zip(self.bones, _bone_ids):
                    p1, p2 = [self.joints[id].pos for id in bone_id]
                    bone.pos = p1
                    bone.axis = p2 - p1
                updated = True
        return updated

# Un hueso es un cilindro que conecta dos articulaciones.
_bone_ids = [[0, 1], [1, 2], [2, 3], [7, 6], [6, 5], [5, 4], [4, 2],
             [2, 8], [8, 9], [9, 10], [10, 11], [15, 14], [14, 13], [13, 12],
             [12, 0], [0, 16], [16, 17], [17, 18], [18, 19]]

# Inicializamos el sensor kinect.
_kinect = nui.Runtime()
_kinect.skeleton_engine.enabled = True
_kinect.camera.elevation_angle = 10

if __name__ == '__main__':
    skeleton = Skeleton(frame(visible=False))
    while True:
        rate(30)
        skeleton.frame.visible = skeleton.update()








