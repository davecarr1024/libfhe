aspects: Graphics.Window
name: window

children:
    pos:
        aspects: Graphics.SceneNode2
        vars:
            pos: Vec2(0.25,0.25)
            scale: Vec2(0.5,0.5)
        children:
            mat:
                aspects: Graphics.Material
                vars:
                    color: 1,0,0
                children:
                    rect:
                        aspects: Graphics.Rect
