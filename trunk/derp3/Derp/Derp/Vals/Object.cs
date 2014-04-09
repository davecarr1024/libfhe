using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class Object : ScopeVal
    {
        public Class Class { get; set; }

        public Scope Scope { get; set; }

        public Object(Class parentClass)
        {
            Class = parentClass;
            Scope = new Scope(Class.Scope);
            foreach (string id in Scope.Keys)
            {
                if (Scope[id] is Func || Scope[id] is Builtin)
                {
                    Scope[id] = new Method(this, Scope[id]);
                }
            }
        }

        public virtual Val Clone()
        {
            return new Object(Class);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            Val call;
            if (Scope.TryGetValue("__call__", out call))
            {
                return call.Apply(args, scope);
            }
            else
            {
                throw new Exception("call on obj with no __call__ method");
            }
        }
    }
}
