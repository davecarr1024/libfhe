using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Sherp
{
    [TestClass]
    public class InterpreterTest
    {
        [TestMethod]
        public void TestRefs()
        {
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval(@"
                class Test
                {
                    void foo( Bool b, Bool c )
                    {
                    }

                    void Main()
                    {
                    }
                }
            "));
        }
    }
}
