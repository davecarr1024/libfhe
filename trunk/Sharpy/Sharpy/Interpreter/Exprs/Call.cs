using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Call : Expr
    {
        public Expr Obj { get; private set; }

        public List<Expr> Args { get; private set; }

        public Call(Expr obj, List<Expr> args)
        {
            Obj = obj;
            Args = args;
        }

        public override Vals.Val Eval(Scope scope)
        {
            Ref r = Obj as Ref;
            if (r != null)
            {
                return r.Resolve(scope).Apply(r.Names.Last(), Args.Select(arg => arg.Eval(scope)).ToArray());
            }
            else
            {
                throw new NotImplementedException();
                //return Obj.Eval(scope).Apply(Args.Select(arg => arg.Eval(scope)).ToArray());
            }
        }

        public override string ToString()
        {
            return string.Format("{0}({1})", Obj, string.Join(", ", Args.Select(arg => arg.ToString()).ToArray()));
        }
    }
}
