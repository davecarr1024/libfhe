using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class Method : Val
    {
        public Object Object { get; set; }

        public Val Func { get; set; }

        public Method(Object obj, Val func)
        {
            Object = obj;
            Func = func;
        }

        public Val Clone()
        {
            return new Method(Object, Func);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            List<Expr> funcArgs = new List<Expr>() { new Exprs.Direct(Object) };
            funcArgs.AddRange(args);
            return Func.Apply(funcArgs, scope);
        }

        public bool AsBool()
        {
            return true;
        }
    }
}
