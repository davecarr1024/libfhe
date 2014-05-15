using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Int : Expr
    {
        public Mods Mods { get { return new Mods(); } }

        public int Value { get; private set; }

        public Int(int value)
        {
            Value = value;
        }

        public Vals.Val Eval(Scope scope)
        {
            return new Vals.Int(Value);
        }
    }
}
