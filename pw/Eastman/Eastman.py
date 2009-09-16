from Core.FSM import FSM

from Splash import Splash
from MainMenu import MainMenu
from Map import Map

class Eastman(FSM):
    def enter_start(self):
        self.transition("splash")
        
    def enter_quit(self):
        self.globalPublish("shutdown")
        
    def enter_splash(self):
        self.splash = Splash(parent = self)
        
    def exit_splash(self):
        self.removeChild(self.splash)
        
    def enter_mainMenu(self):
        self.mainMenu = MainMenu(parent = self)
        
    def exit_mainMenu(self):
        self.removeChild(self.mainMenu)
        
    def enter_map(self):
        self.map = Map(vars = dict(firstRoom = "Eastman/TestMap/Start.room", firstRoomPos = (10,10)))
        self.addChild(self.map)
        
    def exit_map(self):
        self.removeChild(self.map)
