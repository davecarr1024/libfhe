using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class RefExpr : Expr
    {
        public List<string> Ids { get; set; }

        public RefExpr(params string[] ids)
        {
            Ids = ids.ToList();
        }

        public Scope Resolve(Scope parent)
        {
            Scope scope = parent;
            for (int i = 0; i < Ids.Count - 1; ++i)
            {
                Val val;
                if (scope.Vals.TryGetValue(Ids[i], out val))
                {
                    if (val is ObjectVal)
                    {
                        scope = (val as ObjectVal).Scope;
                    }
                    else
                    {
                        throw new Exception("invalid id " + Ids[i]);
                    }
                }
                else
                {
                    throw new Exception("unknown id " + Ids[i]);
                }
            }
            return scope;
        }

        public Val Eval(Scope parent)
        {
            Scope scope = Resolve(parent);
            Val val;
            if (scope.Vals.TryGetValue(Ids.Last(), out val))
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
