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

        public Func Func { get; set; }

        public Method(Object obj, Func func)
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
            if ( Func.Params.Count < 1 )
            {
                throw new Exception("methods must have one more params" );
            }
            else
            {
                Scope funcScope = scope.Clone();
                funcScope.Vals[Func.Params[0]] = Object;
                List<Expr> funcArgs = new List<Expr>() { new Exprs.Ref(Func.Params[0]) };
                funcArgs.AddRange(args);
                return Func.Apply(funcArgs, funcScope);
            }
        }
    }
}
