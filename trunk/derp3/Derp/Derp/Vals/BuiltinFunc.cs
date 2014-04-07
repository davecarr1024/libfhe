using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class BuiltinFunc : Val
    {
        public Func<List<Expr>, Scope, Val> Func { get; set; }

        public BuiltinFunc(Func<List<Expr>, Scope, Val> func)
        {
            Func = func;
        }

        public Val Clone()
        {
            return new BuiltinFunc(Func);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            return Func(args, scope);
        }
    }
}
