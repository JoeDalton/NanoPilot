from math import pi, sin, cos


from direct.showbase.ShowBase import ShowBase

from direct.task import Task

from direct.actor.Actor import Actor

from direct.interval.IntervalGlobal import Sequence

from panda3d.core import Point3, Quat


import serial


class MyApp(ShowBase):

    def __init__(self):

        ShowBase.__init__(self)


        # Disable the camera trackball controls.

        # self.disableMouse()


        # Load the environment model.

        self.scene = self.loader.loadModel("models/environment")

        # Reparent the model to render.

        self.scene.reparentTo(self.render)

        # Apply scale and position transforms on the model.

        self.scene.setScale(0.25, 0.25, 0.25)

        self.scene.setPos(-8, 42, 0)


        # Load and transform the panda actor.

        self.pandaActor = self.loader.loadModel("models/panda-model")

        self.pandaActor.setScale(0.005, 0.005, 0.005)

        self.pandaActor.reparentTo(self.render)

        self.serial_port = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
        
        self.task_mgr.add(self.update_panda, "update_panda")



    def update_panda(self, task):
        line = self.serial_port.readline().decode("Ascii")
        items = line.split(" ")
        print(items)
        try:
            assert len(items) == 5
            for i in range(5):
                items[i] = float(items[i])
            altitude = items.pop(4)
            quat = Quat(*items)
        except:
            altitude = 0
            quat = Quat(1, 0, 0, 0)
        self.pandaActor.setQuat(quat)
        self.pandaActor.setPos(-8, -42, altitude)
        print(altitude)
        print(items)
        return Task.cont


app = MyApp()

app.run()
