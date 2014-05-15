using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Decl : Expr
    {
        public string Type { get; private set; }

        public string Name { get; private set; }

        public Expr Val { get; private set; }

        public List<Expr> Args { get; private set; }

        public Decl(string type, string name, Expr val, List<Expr> args)
        {
            Type = type;
            Name = name;
            Val = val;
            Args = args;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val type = scope.Get(Type);
            Vals.Val val;
            if (Args != null)
            {
                val = type.Apply(Args.Select(arg => arg.Eval(scope)).ToArray());
            }
            else if (Val != null)
            {
                val = Val.Eval(scope);
            }
            else
            {
                val = type.Apply();
            }
            scope.Add(type, Name, val);
            return val;
        }
    }
}
