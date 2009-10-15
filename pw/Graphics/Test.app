type: Graphics.Window

children:
    camera:
        type: Graphics.Prims3.Camera
        vars:
            pos: =Vec3(5,5,5) + Vec3(1,1,1) * 2 * math.cos(self.getRoot().getVar("time",0)*2)
            lookAt: Vec3(0,0,0)
        children:
            cube:
                type: Graphics.Prims3.Cube
                vars:
                    pos: Vec3(0,0,0)
                    rot: =Quat.fromAxisAngle(Vec3(0,1,0),self.getRoot().getVar("time",0)*10)
                    material:
                        color: 0,0,1
                        texture: raptor.jpg

    circle:
        type: Graphics.Prims2.Circle
        vars:
            pos: =self.getWindow().getVar("mousePos",Vec2())
            scale: Vec2(0.1,0.1)
            rot: =self.getRoot().getVar("time",0) * -2
            slices: 25
            material:
                color: 1,0,0
                texture: penguin.jpg
    textbox:
        type: Graphics.Gui.TextBox
        vars:
            pos: Vec2(0.1,0.7)
            scale: Vec2(0.5,0.1)

    testButton:
        type: Graphics.Gui.Button
        vars:
            pos: Vec2(0.1,0.8)
            scale: Vec2(0.5,0.1)
            text: button
            fill:
                texture: raptor.jpg
                color: 1,1,1,0.1
                
    spinner:
        type: Graphics.Gui.Spinner
        vars:
            pos: Vec2(0.1,0.6)
            scale: Vec2(0.5,0.1)
            items: range(10)
