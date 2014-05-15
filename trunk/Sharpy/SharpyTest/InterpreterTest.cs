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
            Assert.AreEqual(new Interpreter.Vals.NoneType(), Interpreter.Interpreter.Eval("return none;"));
            Assert.AreEqual(new Interpreter.Vals.NoneType(), Interpreter.Interpreter.Eval("return NoneType();"));
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval("return false;"));
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval("return true;"));
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval("return bool(false);"));
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval("return bool(true);"));
            Assert.AreEqual(new Interpreter.Vals.Int(12), Interpreter.Interpreter.Eval("return 12;"));
            Assert.AreEqual(new Interpreter.Vals.Int(14), Interpreter.Interpreter.Eval("return int(14);"));
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
        public void Declarations()
        {
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval("bool b; return b;"));
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval("bool b = true; return b;"));
            Assert.AreEqual(new Interpreter.Vals.Bool(true), Interpreter.Interpreter.Eval("bool b( true ); return b;"));
            Assert.AreEqual(new Interpreter.Vals.Int(0), Interpreter.Interpreter.Eval("int i; return i;"));
            Assert.AreEqual(new Interpreter.Vals.Int(2), Interpreter.Interpreter.Eval("int i = 2; return i;"));
            Assert.AreEqual(new Interpreter.Vals.Int(3), Interpreter.Interpreter.Eval("int i(3); return i;"));
            Assert.AreEqual(new Interpreter.Vals.Str(""), Interpreter.Interpreter.Eval("str s; return s;"));
            Assert.AreEqual(new Interpreter.Vals.Str("foo"), Interpreter.Interpreter.Eval("str s = \"foo\"; return s;"));
            Assert.AreEqual(new Interpreter.Vals.Str("bar"), Interpreter.Interpreter.Eval("str s(\"bar\"); return s;"));
        }

        [TestMethod]
        public void UnaryOperators()
        {
            Assert.AreEqual(new Interpreter.Vals.Bool(false), Interpreter.Interpreter.Eval("return !true;"));
            Assert.AreEqual(new Interpreter.Vals.Int(-1), Interpreter.Interpreter.Eval("return -1;"));
            Assert.AreEqual(new Interpreter.Vals.Int(1), Interpreter.Interpreter.Eval("int a = 0; ++a; return a;"));
            Assert.AreEqual(new Interpreter.Vals.Int(-1), Interpreter.Interpreter.Eval("int a = 0; --a; return a;"));
        }

        [TestMethod]
        public void Assertions()
        {
            try
            {
                Interpreter.Interpreter.Eval("System.Assert(false);");
                Assert.Fail();
            }
            catch (Interpreter.Exceptions.AssertException)
            {
            }
            Interpreter.Interpreter.Eval("System.Assert(true);");
        }

        [TestMethod]
        public void BinaryOperators()
        {
            Interpreter.Interpreter.Eval(@"
                System.Assert( none == NoneType() );
                System.Assert( false == false );
                System.Assert( false != true );
                System.Assert( false == !true );
                System.Assert( true || false );
                System.Assert( false || true );
                System.Assert( !( false || false ) );
                System.Assert( true || true );
                System.Assert( !( false && false ) );
                System.Assert( !( true && false ) );
                System.Assert( !( false && true ) );
                System.Assert( true && true );
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
                int i = 0;
                System.Assert( i == 0 );
                i += 10;
                System.Assert( i == 10 );
                i -= 5;
                System.Assert( i == 5 );
                i *= 6;
                System.Assert( i == 30 );
                i /= 2;
                System.Assert( i == 15 );
            ");
        }

        [TestMethod]
        public void Funcs()
        {
            Interpreter.Interpreter.Eval(@"
                int sq( int i )
                {
                    return i * i;
                }
                System.Assert( sq( 2 ) == 4 );
                System.Assert( sq( 3 ) == 9 );
            ");
        }

        [TestMethod]
        public void StaticMembers()
        {
            Interpreter.Interpreter.Eval(@"
                class foo
                {
                    public static int i = 1;
                }
                System.Assert( foo.i == 1 );
                foo.i = 2;
                System.Assert( foo.i == 2 );
            ");
        }

        [TestMethod]
        public void Classes()
        {
            Interpreter.Interpreter.Eval(@"
                class foo
                {
                    public int j;
                    public void __init__( int i )
                    {
                        j = i;
                    }
                    public foo( int i, int k )
                    {
                        j = ( i * k );
                    }
                    public Int mul( int m )
                    {
                        return j * m;
                    }
                    public bool eq( foo f )
                    {
                        return j == f.j;
                    }
                }
                foo f(2);
                System.Assert( f.mul(4) == 8 );
                f = foo( 5, 2 );
                System.Assert( f.mul(2) == 20 );
                System.Assert( f.eq( foo(10) ) );
            ");
        }
    }
}
