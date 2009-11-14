from Core.Aspect import Aspect

class TestAspect(Aspect):
    def foo(self):
        return "bar"

    def msg_msgTest(self, val):
        self.entity.setVar("msgTestVal",val)

    def set_setTest(self, val):
        self.entity.setVar("setTestVal",val)

    def get_getTest(self):
        return "got"
