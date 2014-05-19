using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Sharpy
{
    [TestClass]
    public class InterpreterTest
    {
        [TestMethod]
        public void Builtins()
        {
            Interpreter.Interpreter.Eval(@"
                System.Assert( None == NoneType() );
                System.Assert( true == bool(true) );
                System.Assert( 3 == int(3) );
                System.Assert( ""foo"" == str(""foo"") );
            ");
        }

        [TestMethod]
        public void Decls()
        {
            Interpreter.Interpreter.Eval(@"
                int a = 3;
                System.Assert( a == 3 );
                int b( 2 );
                System.Assert( b == 2 );
            ");
        }
    }
}
