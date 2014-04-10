using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class SystemBuiltin : Val
    {
        public Func<List<Expr>, Scope, Val> Func { get; set; }

        public bool IsReturn { get; set; }

        public SystemBuiltin(Func<List<Expr>, Scope, Val> func)
        {
            IsReturn = false;
            Func = func;
        }

        public Val Clone()
        {
            return new SystemBuiltin(Func);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            return Func(args, scope);
        }

        public bool AsBool()
        {
            return true;
        }
    }
}
