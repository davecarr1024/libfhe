using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public class Object : ScopeVal, ApplyVal
    {
        public Val Type { get; private set; }

        public bool IsReturn { get; set; }

        public Scope Scope { get; private set; }

        public Object(Class parent)
        {
            IsReturn = false;
            Type = parent;
            Scope = new Scope(parent.Scope);
            foreach (string key in Scope.Keys)
            {
                Val val = Scope[key];
                if (val is ApplyVal)
                {
                    Scope.Add(key, new Method(this, val as ApplyVal));
                }
            }
        }

        public virtual bool ToBool()
        {
            return true;
        }

        public virtual Val Apply(List<Exprs.Expr> args, Scope scope)
        {
            Val func;
            if (Scope.TryGetValue("__init__", out func) && func is ApplyVal)
            {
                return (func as ApplyVal).Apply(args, scope);
            }
            else
            {
                throw new Exception("unable to call __call__");
            }
        }
    }
}
