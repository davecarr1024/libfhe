using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public class Object : Val
    {
        public override Val Type { get; protected set; }

        public override Scope Scope { get; protected set; }

        public Object(Val type)
        {
            Type = type;
            Scope = new Scope(Type.Scope);
            Scope.Add("this", this);
            if (Type.Body != null)
            {
                foreach (Exprs.Expr expr in Type.Body)
                {
                    expr.Eval(Scope);
                }
            }
        }
    }
}
