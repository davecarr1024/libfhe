using System;
using System.Linq;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Sharpy
{
    [TestClass]
    public class InterpreterTest
    {
        [TestMethod]
        public void Vals()
        {
            Assert.AreEqual(
                new Interpreter.Vals.NoneType(),
                Interpreter.Vals.BuiltinClass.Bind(typeof(Interpreter.Vals.NoneType)).Apply()
            );
            Assert.AreEqual(
                new Interpreter.Vals.Bool(true),
                Interpreter.Vals.BuiltinClass.Bind(typeof(Interpreter.Vals.Bool)).Apply(new Interpreter.Vals.Bool(true))
            );
            Assert.AreEqual(
                new Interpreter.Vals.Int(12),
                Interpreter.Vals.BuiltinClass.Bind(typeof(Interpreter.Vals.Int)).Apply(new Interpreter.Vals.Int(12))
            );
            Assert.AreEqual(
                new Interpreter.Vals.Str("foo"),
                Interpreter.Vals.BuiltinClass.Bind(typeof(Interpreter.Vals.Str)).Apply(new Interpreter.Vals.Str("foo"))
            );
        }

        [TestMethod]
        public void BuiltinCtors()
        {
            Assert.AreEqual(new Interpreter.Vals.NoneType(), Interpreter.Interpreter.Eval("return None;"));
            Assert.AreEqual(new Interpreter.Vals.NoneType(), Interpreter.Interpreter.Eval("return NoneType();"));
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval("return False;"));
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval("return True;"));
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval("return Bool(False);"));
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval("return Bool(True);"));
            Assert.AreEqual(new Interpreter.Vals.Int(12), Interpreter.Interpreter.Eval("return 12;"));
            Assert.AreEqual(new Interpreter.Vals.Int(14), Interpreter.Interpreter.Eval("return Int(14);"));
            Assert.AreEqual(new Interpreter.Vals.Str("foo"), Interpreter.Interpreter.Eval("return \"foo\";"));
            Assert.AreEqual(new Interpreter.Vals.Str("bar"), Interpreter.Interpreter.Eval("return Str(\"bar\");"));
        }

        [TestMethod]
        public void MethodBinding()
        {
            Interpreter.Vals.Bool b1 = new Interpreter.Vals.Bool(true);
            Interpreter.Vals.Bool b2 = new Interpreter.Vals.Bool(false);
            Assert.AreNotEqual(
                (b1.Scope.Get("__not__") as Interpreter.Vals.BuiltinFunc).Obj,
                (b2.Scope.Get("__not__") as Interpreter.Vals.BuiltinFunc).Obj
            );
        }

        [TestMethod]
        public void Declaration()
        {
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval("Bool b; return b;"));
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval("Bool b = True; return b;"));
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval("Bool b( True ); return b;"));
            Assert.AreEqual(new Interpreter.Vals.Int(0), Interpreter.Interpreter.Eval("Int i; return i;"));
            Assert.AreEqual(new Interpreter.Vals.Int(2), Interpreter.Interpreter.Eval("Int i = 2; return i;"));
            Assert.AreEqual(new Interpreter.Vals.Int(3), Interpreter.Interpreter.Eval("Int i(3); return i;"));
            Assert.AreEqual(new Interpreter.Vals.Str(""), Interpreter.Interpreter.Eval("Str s; return s;"));
            Assert.AreEqual(new Interpreter.Vals.Str("foo"), Interpreter.Interpreter.Eval("Str s = \"foo\"; return s;"));
            Assert.AreEqual(new Interpreter.Vals.Str("bar"), Interpreter.Interpreter.Eval("Str s(\"bar\"); return s;"));
        }

        [TestMethod]
        public void UnaryOperators()
        {
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval("return !True;"));
            Assert.AreEqual(new Interpreter.Vals.Int(-1), Interpreter.Interpreter.Eval("return -1;"));
            Assert.AreEqual(new Interpreter.Vals.Int(1), Interpreter.Interpreter.Eval("Int a = 0; ++a; return a;"));
            Assert.AreEqual(new Interpreter.Vals.Int(-1), Interpreter.Interpreter.Eval("Int a = 0; --a; return a;"));
        }

        [TestMethod]
        public void Assertion()
        {
            try
            {
                Interpreter.Interpreter.Eval("System.Assert(False);");
                Assert.Fail();
            }
            catch (Interpreter.Exceptions.AssertException)
            {
            }
            Interpreter.Interpreter.Eval("System.Assert(True);");
        }

        [TestMethod]
        public void BinaryOperators()
        {
            Interpreter.Interpreter.Eval(@"
                System.Assert( None == NoneType() );
                System.Assert( False == False );
                System.Assert( False != True );
                System.Assert( False == !True );
                System.Assert( True || False );
                System.Assert( False || True );
                System.Assert( !( False || False ) );
                System.Assert( True || True );
                System.Assert( !( False && False ) );
                System.Assert( !( True && False ) );
                System.Assert( !( False && True ) );
                System.Assert( True && True );
                System.Assert( 1 == 1 );
                System.Assert( 1 != 2 );
                System.Assert( ( 1 + 2 ) == 3 );
                System.Assert( ( 1 - 2 ) == -1 );
                System.Assert( ( 1 * -2 ) == -2 );
                System.Assert( ( 10 / 2 ) == 5 );
                System.Assert( 1 < 2 );
                System.Assert( 2 <= 2 );
                System.Assert( 2 > 1 );
                System.Assert( 2 >= 2 );
            ");
        }

        [TestMethod]
        public void Func()
        {
            Interpreter.Interpreter.Eval(@"
                Int sq( Int i )
                {
                    return i * i;
                }
                System.Assert( sq( 2 ) == 4 );
                System.Assert( sq( 3 ) == 9 );
            ");
        }
    }
}
