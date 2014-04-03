using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using derp3;

namespace derp3test
{
    [TestClass]
    public class LispTest
    {
        [TestMethod]
        public void TestLisp()
        {
            Assert.AreEqual(Lisp.Eval("10"), new Lisp.Value(10));
            Assert.AreEqual(Lisp.Eval("3.14"), new Lisp.Value(3.14f));
            Assert.AreEqual(Lisp.Eval("\"foo\""), new Lisp.Value("foo"));
            Assert.AreEqual(
                Lisp.Eval(
                    "x",
                    new Dictionary<string, Lisp.Value>()
                    { 
                        {"x", new Lisp.Value(2)}
                    }),
                new Lisp.Value(2));
            Assert.AreEqual(Lisp.Eval("(define y 3) y"), new Lisp.Value(3));
        }
    }
}
