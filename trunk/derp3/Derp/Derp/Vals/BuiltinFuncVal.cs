using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class BuiltinFuncVal : Val
    {
        public Func<List<Expr>, Scope, Val> Func { get; set; }

        public BuiltinFuncVal(Func<List<Expr>, Scope, Val> func)
        {
            Func = func;
        }

        public Val Clone()
        {
            return new BuiltinFuncVal(Func);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            return Func(args, scope);
        }
    }
}
