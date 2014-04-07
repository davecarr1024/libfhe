using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using derp3;

namespace derp3test
{
    [TestClass]
    public class DerpTest
    {
        [TestMethod]
        public void TestLiterals()
        {
            Assert.AreEqual(new NoneVal(), Derp.Eval("None;"));
            Assert.AreEqual(new BoolVal(true), Derp.Eval("True;"));
            Assert.AreEqual(new IntVal(2), Derp.Eval("2;"));
            Assert.AreEqual(new FloatVal(2.1f), Derp.Eval("2.1;"));
            Assert.AreEqual(new StringVal("bar"), Derp.Eval(@"""bar"";"));
        }

        [TestMethod]
        public void TestFuncs()
        {
            Assert.AreEqual(new IntVal(3), Derp.Eval(@"
                def foo(a) { a; } foo(3);
            "));
            Assert.AreEqual(new IntVal(7), Derp.Eval(@"
                def foo(a,b) { a + b; } foo(3,4);
            "));
        }

        [TestMethod]
        public void TestAssignment()
        {
            Assert.AreEqual(new StringVal("hello"), Derp.Eval(@"
                a = ""hello"";
                a;
            "));
        }

        [TestMethod]
        public void TestClass()
        {
            Assert.AreEqual(new IntVal(15), Derp.Eval(@"
                class foo 
                {
                    def init( self, a )
                    {
                        self.baz = a;
                    }
                    def bar( self, b )
                    {
                        self.baz - b;
                    }
                }
                f = foo();
                f.init(20);
                f.bar( 5 );
            "));
        }
    }
}
