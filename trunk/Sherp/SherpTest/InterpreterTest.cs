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

        [TestMethod]
        public void TestAssignment()
        {
            Interpreter.Interpreter.Eval(@"
                class Test
                {
                    void Main()
                    {
                        Bool b = True;
                        System.Assert(b);
                    }
                }
            ");
        }

        [TestMethod]
        public void TestBinaryOperators()
        {
            Interpreter.Interpreter.Eval(@"
                class Test
                {
                    void Main()
                    {
                        Bool b = False;
                        System.Assert(b == False);
                        System.Assert(b != True);
                        System.Assert(None != b);
                        System.Assert(None == None);
                        System.Assert(1 == 1);
                        System.Assert(1 != 2);
                        System.Assert((1 + 1) == 2);
                        System.Assert((10 - 2) == 8);
                        System.Assert((3 * 2) == 6);
                        System.Assert((20 / 4) == 5);
                        System.Assert(1 < 2);
                        System.Assert(1 <= 2);
                        System.Assert(1 <= 1);
                        System.Assert(2 > 1);
                        System.Assert(2 >= 1);
                        System.Assert(2 >= 2);
                    }
                }
            ");
        }

        [TestMethod]
        public void TestBuiltinConstructors()
        {
            Interpreter.Interpreter.Eval(@"
                class Test
                {
                    void Main()
                    {
                        System.Assert( !Bool() );
                        System.Assert( Bool( True ) );
                        System.Assert( !Bool( False ) );
                        System.Assert( Int(3) == 3 );
                    }
                }
            ");
        }
    }
}
