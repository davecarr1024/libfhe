using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Ref : Expr
    {
        public List<string> Ids { get; private set; }

        public Ref(string id, params string[] ids)
        {
            Ids = new List<string>() { id };
            Ids.AddRange(ids);
        }

        public Scope Resolve(Scope scope)
        {
            foreach (string id in Ids.Take(Ids.Count - 1))
            {
                Vals.Val val = scope.Get(id);
                if (val.Scope != null)
                {
                    scope = val.Scope;
                }
                else
                {
                    throw new Exception("trying to get child " + id + " from non-scope val " + val);
                }
            }
            return scope;
        }

        public Vals.Val Eval(Scope scope)
        {
            return Resolve(scope).Get(Ids.Last());
        }

        public override string ToString()
        {
            return string.Join(".", Ids.ToArray());
        }
    }
}
