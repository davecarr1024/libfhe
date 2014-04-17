using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public class BuiltinMethod : ApplyVal
    {
        public Val Type { get { return Class.Bind(GetType()); } }

        public bool IsReturn { get; set; }

        public Func<List<Exprs.Expr>, Scope, Val> Func { get; private set; }

        public BuiltinMethod(Func<List<Exprs.Expr>, Scope, Val> func)
        {
            IsReturn = false;
            Func = func;
        }

        public Val Apply(List<Exprs.Expr> args, Scope scope)
        {
            return Func(args, scope);
        }

        public bool ToBool()
        {
            return true;
        }
    }
}
