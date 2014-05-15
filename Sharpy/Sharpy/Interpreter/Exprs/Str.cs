using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Str : Expr
    {
        public Mods Mods { get { return new Mods(); } }

        public string Value { get; private set; }

        public Str(string value)
        {
            Value = value;
        }

        public Vals.Val Eval(Scope scope)
        {
            return new Vals.Str(Value);
        }
    }
}
