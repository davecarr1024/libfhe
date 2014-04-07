using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public class MethodVal : Val
    {
        public ObjectVal Object { get; set; }

        public FuncVal Func { get; set; }

        public MethodVal(ObjectVal obj, FuncVal func)
        {
            Object = obj;
            Func = func;
        }

        public Val Clone()
        {
            return new MethodVal(Object, Func);
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
                List<Expr> funcArgs = new List<Expr>() { new RefExpr(Func.Params[0]) };
                funcArgs.AddRange(args);
                return Func.Apply(funcArgs, funcScope);
            }
        }
    }
}
