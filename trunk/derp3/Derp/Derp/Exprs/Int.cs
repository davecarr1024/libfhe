using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class Int : Expr
    {
        public int Value { get; set; }

        public Int(int value)
        {
            Value = value;
        }

        public Val Eval(Scope scope)
        {
            return new Vals.Int(Value);
        }

        public override string ToString()
        {
            return Value.ToString();
        }
    }
}
