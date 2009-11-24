from Core.App import app
from Core.Vec2 import Vec2

from Graphics.Window import window
from Graphics.Rect import Rect
from Graphics.Material import Material

from Physics.World import world
from Physics.Body import body

r = Rect()
r.material = Material(color = (1,0,0))
r.pos = Vec2(0.25,0.25)
r.scale = Vec2(0.25,0.25)
window.root2.addChild(r)

app.run()
