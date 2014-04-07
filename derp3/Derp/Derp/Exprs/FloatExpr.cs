using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class FloatExpr : Expr
    {
        public float Value { get; set; }

        public FloatExpr(float value)
        {
            Value = value;
        }

        public Val Eval(Scope scope)
        {
            return new FloatVal(Value);
        }
    }
}
