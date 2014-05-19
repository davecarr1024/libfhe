using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Ref : Expr
    {
        public List<string> Names { get; private set; }

        public Ref(string name, params string[] names)
        {
            Names = new List<string>() { name };
            Names.AddRange(names);
        }

        public Scope Resolve(Scope scope)
        {
            foreach (string name in Names.Take(Names.Count - 1))
            {
                Vals.Val val = scope.Get(name);
                if (val.Scope != null)
                {
                    scope = val.Scope;
                }
                else
                {
                    throw new Exception("trying to get ref from non-scope val " + val);
                }
            }
            return scope;
        }

        public override Vals.Val Eval(Scope scope)
        {
            return Resolve(scope).Get(Names.Last());
        }

        public override string ToString()
        {
            return string.Join(".", Names.ToArray());
        }
    }
}
