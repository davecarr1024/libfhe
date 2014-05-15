using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Decl : Expr
    {
        public Expr Type { get; private set; }

        public Mods Mods { get; private set; }

        public string Name { get; private set; }

        public Expr Val { get; private set; }

        public List<Expr> Args { get; private set; }

        public Decl(Mods mods, Expr type, string name, Expr val, List<Expr> args)
        {
            Mods = mods;
            Type = type;
            Name = name;
            Val = val;
            Args = args;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val type = Type.Eval(scope);
            Vals.Val val;
            if (Args != null)
            {
                val = type.Apply(Args.Select(arg => arg.Eval(scope)).ToArray());
            }
            else if (Val != null)
            {
                val = Interpreter.Convert(Val.Eval(scope), type);
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
