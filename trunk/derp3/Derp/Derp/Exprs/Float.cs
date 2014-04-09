using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class Float : Expr
    {
        public float Value { get; set; }

        public Float(float value)
        {
            Value = value;
        }

        public Val Eval(Scope scope)
        {
            return new Vals.Float(Value);
        }

        public override string ToString()
        {
            return Value.ToString();
        }
    }
}
