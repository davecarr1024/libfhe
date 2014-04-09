using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class Call : Expr
    {
        public Expr Func { get; set; }

        public List<Expr> Args { get; set; }

        public Call(Expr func, List<Expr> args)
        {
            Func = func;
            Args = args;
        }

        public Val Eval(Scope scope)
        {
            return Func.Eval(scope).Apply(Args, scope);
        }

        public override string ToString()
        {
            return Func + "(" + string.Join(", ", Args.Select(arg => arg.ToString()).ToArray()) + ")";
        }
    }
}
