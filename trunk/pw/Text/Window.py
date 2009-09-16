# -*- coding: utf-8 -*-
import curses

from SceneNode import SceneNode

class Window(SceneNode):
    def __init__(self, **args):
        self.colors = {}
        self.screen = None
        self.focus = None
        
        SceneNode.__init__(self, **args)
        
        def box():
            if self.screen:
                y,x = self.screen.getmaxyx()
                return 0,0,x,y
            else:
                return 0,0,0,0
                
        self.setVar("box",box)
        
    def onAttach(self):
        if not self.screen:
            self.screen = curses.initscr()
            curses.noecho()
            curses.cbreak()
            self.screen.keypad(1)
            self.screen.nodelay(1)
            #curses.start_color()
    
    def msg_shutdown(self):
        self.shutdown()
    
    def onDetach(self):
        self.shutdown()
        
    def shutdown(self):
        if self.screen:
            curses.nocbreak()
            self.screen.keypad(0)
            curses.echo()
            curses.endwin()
            self.screen = None
            
    def msg_update(self, time, dtime):
        if self.screen:
            ch = self.screen.getch()
            if ch >= 0:
                if ch == self.getVar("quitChar",27):
                    self.globalPublish("shutdown")
                elif ch == self.getVar("incFocusChar",258):
                    self.moveFocus(1)
                elif ch == self.getVar("decFocusChar",259):
                    self.moveFocus(-1)
                else:
                    self.publish("textInput",ch)
        
        if self.screen and time - self.getVar("lastRenderTime",0) > 1.0 / self.getVar("fps",60):
            self.setVar("lastRenderTime",time)
            
            self.screen.clear()
            context = Window.RenderContext(self)
            self.publish("textRender",context)
            context.flush()
            self.screen.refresh()
            
    def getWidgets(self):
        widgets = self.filterDescendents(lambda node: isinstance(node,Widget), True)
        def key(node):
            x,y = node.getVar("globalPos",(0,0))
            return y * self.getVar("box")[2] + x
        widgets.sort(key = key)
        return widgets
    
    def moveFocus(self, d):
        widgets = self.getWidgets()
        if widgets:
            if self.focus in widgets:
                self.setFocus(widgets[(widgets.index(self.focus)+d)%len(widgets)])
            else:
                self.setFocus(widgets[0])
        else:
            self.setFocus(None)
            
    def setFocus(self, focus):
        if focus != self.focus:
            if self.focus:
                self.focus.publish("unfocus")
            self.focus = focus
            if self.focus:
                self.focus.publish("focus")
            
    class RenderContext:
        def __init__(self, window):
            self.window = window
            
            self.buffer = {}
            
        def flush(self):
            for (x,y), (layer,s,fg,bg) in self.buffer.iteritems():
                self.window.screen.addstr(y,x,s)#,self.getColor(fg,bg))
            
        def add(self, x, y, layer, s, fg, bg):
            assert len(s) == 1
            minx,miny,screenw,screenh = self.window.getVar("box")
            if x >= 0 and x < screenw and y >= 0 and y < screenh and not (x == screenw-1 and y == screenh-1):
                if (x,y) not in self.buffer or layer >= self.buffer[x,y][0]:
                    self.buffer[x,y] = layer,s,fg,bg
        
        def getColor(self, fg, bg):
            if (fg,bg) not in self.window.colors:
                curses_fg = getattr(curses,"COLOR_%s" % fg.upper())
                curses_bg = getattr(curses,"COLOR_%s" % bg.upper())
                i = len(self.window.colors) + 1
                curses.init_pair(i,curses_fg,curses_bg)
                self.window.colors[fg,bg] = i
            return curses.color_pair(self.window.colors[fg,bg])

from Widget import Widget
