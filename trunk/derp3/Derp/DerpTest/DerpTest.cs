using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Derp
{
    [TestClass]
    public class DerpTest
    {
        [TestMethod]
        public void TestLiterals()
        {
            Assert.AreEqual(new Vals.None(), Derp.Eval("None;"));
            Assert.AreEqual(new Vals.Bool(true), Derp.Eval("True;"));
            Assert.AreEqual(new Vals.Int(2), Derp.Eval("2;"));
            Assert.AreEqual(new Vals.Float(2.1f), Derp.Eval("2.1;"));
            Assert.AreEqual(new Vals.String("bar"), Derp.Eval(@"""bar"";"));
        }

        [TestMethod]
        public void TestFuncs()
        {
            Assert.AreEqual(new Vals.Int(3), Derp.Eval(@"
                def foo(a) { a; } foo(3);
            "));
            Assert.AreEqual(new Vals.Int(7), Derp.Eval(@"
                def foo(a,b) { a + b; } foo(3,4);
            "));
        }

        [TestMethod]
        public void TestAssignment()
        {
            Assert.AreEqual(new Vals.String("hello"), Derp.Eval(@"
                a = ""hello"";
                a;
            "));
        }

        [TestMethod]
        public void TestClass()
        {
            Assert.AreEqual(new Vals.Int(15), Derp.Eval(@"
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

        [TestMethod]
        public void TestFile()
        {
            Assert.AreEqual(new Vals.Int(3), Derp.Eval(System.IO.File.ReadAllText("test.derp")));
        }
    }
}
