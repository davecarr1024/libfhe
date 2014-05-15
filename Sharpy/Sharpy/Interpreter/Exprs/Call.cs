using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Call : Expr
    {
        public Mods Mods { get { return new Mods(); } }

        public Expr Obj { get; private set; }

        public List<Expr> Args { get; private set; }

        public Call(Expr obj, List<Expr> args)
        {
            Obj = obj;
            Args = args;
        }

        public Vals.Val Eval(Scope scope)
        {
            if (Obj is Ref)
            {
                return (Obj as Ref).Resolve(scope).Apply((Obj as Ref).Ids.Last(), Args.Select(arg => arg.Eval(scope)).ToArray());
            }
            else
            {
                return Obj.Eval(scope).Apply(Args.Select(arg => arg.Eval(scope)).ToArray());
            }
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
