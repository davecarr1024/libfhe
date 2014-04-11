using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Vals
{
    public class Builtin : CallVal
    {
        public Func<List<Expr>, Scope, Val> Func { get; set; }

        public bool IsReturn { get; set; }

        public Builtin(Func<List<Expr>, Scope, Val> func)
        {
            IsReturn = true;
            Func = func;
        }

        public Val Call(List<Expr> args, Scope scope)
        {
            return Func(args, scope);
        }

        public bool ToBool(Scope scope)
        {
            return true;
        }
    }
}
