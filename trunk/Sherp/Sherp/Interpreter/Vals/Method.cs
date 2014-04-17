using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public class Method : ApplyVal
    {
        public Val Type { get { return Class.Bind(GetType()); } }

        public bool IsReturn { get; set; }

        public Val Obj { get; private set; }

        public ApplyVal Func { get; private set; }

        public Method(Val obj, ApplyVal func)
        {
            IsReturn = false;
            Obj = obj;
            Func = func;
        }

        public Val Apply(List<Exprs.Expr> args, Scope scope)
        {
            List<Exprs.Expr> funcArgs = new List<Exprs.Expr>() { new Exprs.Direct(Obj) };
            funcArgs.AddRange(args);
            return Func.Apply(funcArgs, scope);
        }

        public bool ToBool()
        {
            return true;
        }
    }
}
