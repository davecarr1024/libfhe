using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Exprs
{
    public class Ref : Expr
    {
        public List<string> Ids { get; private set; }

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
                    ScopeVal scopeVal = val as ScopeVal;
                    if (scopeVal != null)
                    {
                        scope = scopeVal.Scope;
                    }
                    else
                    {
                        throw new Exception("invalid ref " + id);
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
            if (scope.TryGetValue(Ids.Last(), out val))
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
