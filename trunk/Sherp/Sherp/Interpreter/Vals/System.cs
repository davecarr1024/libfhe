using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class System : Object
    {
        public System()
            : base(Class.Bind(typeof(System)))
        {
        }

        [Attrs.SystemMethod]
        public static Val Assert(List<Exprs.Expr> args, Scope scope)
        {
            foreach (Exprs.Expr arg in args)
            {
                if (!arg.Eval(scope).ToBool())
                {
                    throw new Exception("Assertion failed: " + arg);
                }
            }
            return new NoneType();
        }
    }
}
