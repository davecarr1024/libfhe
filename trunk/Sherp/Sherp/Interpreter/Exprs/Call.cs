using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class Call : Expr
    {
        public Expr Func { get; private set; }

        public List<Expr> Args { get; private set; }

        public Call(Expr func, List<Expr> args)
        {
            Func = func;
            Args = args;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val func = Func.Eval(scope);
            if (func is Vals.ApplyVal)
            {
                return (func as Vals.ApplyVal).Apply(Args, scope);
            }
            else
            {
                throw new Exception("unable to call uncallable func " + func);
            }
        }

        public override string ToString()
        {
            return Func + "(" + string.Join(", ", Args.Select(arg => arg.ToString()).ToArray()) + ")";
        }
    }
}
