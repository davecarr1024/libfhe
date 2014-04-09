using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class Ref : Expr
    {
        public List<string> Ids { get; set; }

        public Ref(params string[] ids)
        {
            Ids = ids.ToList();
        }

        public Scope Resolve(Scope scope)
        {
            foreach (string id in Ids.Take(Ids.Count - 1))
            {
                Val val;
                if (scope.TryGetValue(id, out val))
                {
                    if (val is ScopeVal)
                    {
                        scope = (val as ScopeVal).Scope;
                    }
                    else
                    {
                        throw new Exception("invalid scope val " + id);
                    }
                }
                else
                {
                    throw new Exception("unknown id " + id);
                }
            }
            return scope;
        }

        public Val Eval(Scope scope)
        {
            scope = Resolve(scope);
            Val val;
            if ( scope.TryGetValue(Ids.Last(),out val))
            {
                return val;
            }
            else
            {
                throw new Exception("unknown id " + Ids.Last());
            }
        }
    }
}
