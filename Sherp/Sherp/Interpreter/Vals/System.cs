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

        [Attrs.SystemMethod(new Type[] { typeof(Bool) })]
        public static Val Assert(List<Exprs.Expr> args, Scope scope)
        {
            Val val = args[0].Eval(scope);
            if (!val.ToBool())
            {
                throw new Exception("Assertion failed: expr " + args[0] + " val " + val);
            }
            else
            {
                return new NoneType();
            }
        }
    }
}
