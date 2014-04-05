using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using derp3;

namespace derp3test
{
    [TestClass]
    public class DerpTest
    {
        [TestMethod]
        public void TestDerp()
        {
            Assert.AreEqual(new Derp.Val(), Derp.Eval("None;"));
            Assert.AreEqual(new Derp.Val(true), Derp.Eval("True;"));
            Assert.AreEqual(new Derp.Val(2), Derp.Eval("2;"));
            Assert.AreEqual(new Derp.Val(2.1f), Derp.Eval("2.1;"));
            Assert.AreEqual(new Derp.Val("bar"), Derp.Eval("\"bar\";"));
            Assert.AreEqual(new Derp.Val(4), Derp.Eval(@"
                def foo(a,b) { a; b; } foo(3,4);
            "));
            Assert.AreEqual(new Derp.Val(7), Derp.Eval(@"
                def foo(a,b) { a + b; } foo(3,4);
            "));
        }
    }
}
