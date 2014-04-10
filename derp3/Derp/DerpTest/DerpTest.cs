using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Derp
{
    [TestClass]
    public class DerpTest
    {
        [TestMethod]
        public void TestBind()
        {
            Scope scope = Derp.DefaultScope();
            Assert.IsTrue(scope.ContainsKey("Bool"));
            Vals.Class boolClass = scope["Bool"] as Vals.Class;
            Assert.IsTrue(boolClass != null);
            Assert.IsTrue(boolClass.Scope.ContainsKey("__str__"));
            Vals.Object b = new Vals.Bool(true);
            Assert.IsTrue(b.Scope.ContainsKey("__str__"));
            Assert.AreEqual(new Vals.String("True"), b.Scope["__str__"].Apply(new List<Expr>(), scope));
        }

        [TestMethod]
        public void TestLiterals()
        {
            Assert.AreEqual(new Vals.NoneType(), Derp.Eval("None;"));
            Assert.AreEqual(new Vals.Bool(false), Derp.Eval("False;"));
            Assert.AreEqual(new Vals.Bool(true), Derp.Eval("True;"));
            Assert.AreEqual(new Vals.Int(2), Derp.Eval("2;"));
            Assert.AreEqual(new Vals.Float(2.1f), Derp.Eval("2.1;"));
            Assert.AreEqual(new Vals.String("bar"), Derp.Eval(@"""bar"";"));
            Assert.AreEqual(new Vals.String(""), Derp.Eval(@""""";"));
        }

        [TestMethod]
        public void TestFuncs()
        {
            Assert.AreEqual(new Vals.Int(7), Derp.Eval(@"def foo(a,b) { a; return a + b; b; } foo(3,4);"));
        }

        [TestMethod]
        public void TestAssignment()
        {
            Assert.AreEqual(new Vals.String("hello"), Derp.Eval(@"a = ""hello""; a;"));
        }

        [TestMethod]
        public void TestClass()
        {
            Derp.Eval(@"
                class foo 
                {
                    def __init__( self, a )
                    {
                        self.baz = a;
                    }
                    def bar( self, b )
                    {
                        return self.baz - b;
                    }
                }
                f = foo(20);
                Assert( 15 == f.bar( 5 ) );
            ");
        }

        [TestMethod]
        public void TestUnaryOperators()
        {
            Assert.AreEqual(new Vals.Int(-1), Derp.Eval(@"-1;"));
            Assert.AreEqual(new Vals.Bool(false), Derp.Eval(@"!True;"));
            Assert.AreEqual(new Vals.Int(-3), Derp.Eval(@"(-1) * 3;"));
        }

        [TestMethod]
        public void TestBuiltinCtors()
        {
            Assert.AreEqual(new Vals.NoneType(), Derp.Eval(@"NoneType();"));
            Assert.AreEqual(new Vals.Bool(true), Derp.Eval(@"Bool(True);"));
            Assert.AreEqual(new Vals.Int(12), Derp.Eval(@"Int(12);"));
            Assert.AreEqual(new Vals.Float(2.1f), Derp.Eval(@"Float(2.1);"));
            Assert.AreEqual(new Vals.String("bar"), Derp.Eval(@"String(""bar"");"));
        }

        [TestMethod]
        public void TestIf()
        {
            Derp.Eval(@"
                a = 3;
                b = 1;
                if ( a == 3 )
                {
                    b = 2;
                }
                Assert( b == 2 );
                if ( b == 1 )
                {
                    a = 2;
                }
                Assert( a == 3 );
            ");
            Derp.Eval(@"
                def foo(a) 
                { 
                    if ( a == 1 ) 
                    { 
                        return 2; 
                    } 
                    else 
                    { 
                        return 3; 
                    } 
                } 
                Assert( foo( 1 ) == 2 );
                Assert( foo( 0 ) == 3 );
            ");
        }

        [TestMethod]
        public void TestComps()
        {
            Derp.Eval(@"
                Assert( 1 > 0 );
                Assert( 0 < 1 );
                Assert( 0 <= 0 );   
                Assert( 0 <= 1 );
                Assert( 1 >= 1 );
                Assert( 1 >= 0 );
            ");
        }

        [TestMethod]
        public void TestWhile()
        {
            Derp.Eval(@"
                def foo(n)
                {
                    i = 0;
                    r = 1;
                    while ( i < n )
                    {
                        i = ( i + 1 );
                        r = ( r * 2 );
                    }
                    return r;
                }
                Assert( foo(4) == 16 );
                Assert( foo(5) == 32 );
            ");
        }

        [TestMethod]
        public void TestFor()
        {
            Derp.Eval(@"
                def foo(n, s)
                {
                    r = """";
                    for ( i = 0; i < n; i = ( i + 1 ) )
                    {
                        r = ( r + s );
                    }
                    return r;
                }
                Assert( foo(2,""bar"") == ""barbar"" );
            ");
        }

        [TestMethod]
        public void TestFile()
        {
            Derp.Eval(System.IO.File.ReadAllText("test.derp"));
        }
    }
}
