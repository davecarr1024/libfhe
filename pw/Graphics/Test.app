type: Graphics.Window

children:
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
    rect:
        type: Graphics.Prims2.Rect
        vars:
            scale: Vec2(0.5,1)
            filled: False
            material:
                color: 0,0,1
        children:
            text:
                type: Graphics.Prims2.Text
                vars:
                    text: hey
                    material:
                        color: 0,0,0
                        
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
