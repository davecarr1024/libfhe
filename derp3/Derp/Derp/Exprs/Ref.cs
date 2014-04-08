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
            Val val = null;
            foreach (string id in Ids.Take(Ids.Count - 1))
            {
                if (scope.Vals.TryGetValue(id, out val))
                {
                    scope = val;
                }
                else
                {
                    throw new Exception("unknown id " + id);
                }
            }
            if (val != null)
            {
                return val;
            }
            else
            {
                return scope;
            }
        }

        public Val Eval(Scope scope)
        {
            Val val = null;
            foreach (string id in Ids)
            {
                if (scope.Vals.TryGetValue(id, out val))
                {
                    scope = val;
                }
                else
                {
                    throw new Exception("unknown id " + id);
                }
            }
            if (val != null)
            {
                return val;
            }
            else
            {
                throw new Exception("invalid ids");
            }
        }
    }
}
