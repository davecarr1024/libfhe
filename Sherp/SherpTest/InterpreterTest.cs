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
                    Bool Main()
                    {
                        return True;
                    }
                }
            "));
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval(@"
                class Test
                {
                    Bool Main()
                    {
                        return False;
                    }
                }
            "));
            Assert.AreEqual(new Interpreter.Vals.NoneType(), Interpreter.Interpreter.Eval(@"
                class Test
                {
                    void Main()
                    {
                        return;
                    }
                }
            "));
        }
    }
}
