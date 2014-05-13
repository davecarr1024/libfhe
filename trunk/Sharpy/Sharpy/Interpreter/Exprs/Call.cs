using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Call : Expr
    {
        public Ref Obj { get; private set; }

        public List<Expr> Args { get; private set; }

        public Call(Ref obj, List<Expr> args)
        {
            Obj = obj;
            Args = args;
        }

        public Vals.Val Eval(Scope scope)
        {
            return Obj.Resolve(scope).Apply(Obj.Ids.Last(), Args.Select(arg => arg.Eval(scope)).ToList());
        }

        public override string ToString()
        {
            if (Args.Any())
            {
                return string.Format("{0}( {1} )", Obj, string.Join(", ", Args.Select(arg => arg.ToString()).ToArray()));
            }
            else
            {
                return string.Format("{0}()", Obj);
            }
        }
    }
}
