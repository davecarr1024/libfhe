using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class System : Object
    {
        public System()
            : base(BuiltinClass.Bind(typeof(System)))
        {
        }

        [Attrs.BuiltinFunc(true)]
        public static void Assert(List<Exprs.Expr> exprs, Scope scope)
        {
            foreach (Exprs.Expr expr in exprs)
            {
                Val val = expr.Eval(scope);
                if (!(BuiltinClass.Bind(typeof(Bool)).Apply(new List<Val>() { val }) as Bool).Value)
                {
                    throw new Exceptions.AssertException(expr.ToString());
                }
            }
        }
    }
}
