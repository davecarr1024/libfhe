using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public class CallExpr : Expr
    {
        public Expr Func { get; set; }

        public List<Expr> Args { get; set; }

        public CallExpr(Expr func, List<Expr> args)
        {
            Func = func;
            Args = args;
        }

        public Val Eval(Scope scope)
        {
            return Func.Eval(scope).Apply(Args, scope);
        }
    }
}
