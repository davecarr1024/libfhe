using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
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
                Vals.Val val;
                if (!scope.TryGetValue(id, out val))
                {
                    throw new Exception("unknown id " + id);
                }
                else if (!(val is Vals.ScopeVal))
                {
                    throw new Exception("invalid id " + id);
                }
                else
                {
                    scope = (val as Vals.ScopeVal).Scope;
                }
            }
            return scope;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val val;
            if (Resolve(scope).TryGetValue(Ids.Last(), out val))
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
