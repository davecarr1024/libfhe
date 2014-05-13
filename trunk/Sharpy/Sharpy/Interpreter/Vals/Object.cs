using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public class Object : Val
    {
        public Val Type { get; private set; }

        public Scope Scope { get; private set; }

        public List<Exprs.Expr> Body { get { return null; } }

        public bool IsReturn { get; set; }

        public Object(Val type)
        {
            IsReturn = false;
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

        public bool CanApply(List<Exprs.Expr> exprs, List<Val> args)
        {
            return Scope.CanApply("__call__", args);
        }

        public Val Apply(List<Exprs.Expr> exprs, List<Val> args)
        {
            return Scope.Apply("__call__", args);
        }

        public override string ToString()
        {
            return string.Format("Object({0})", Type);
        }
    }
}
