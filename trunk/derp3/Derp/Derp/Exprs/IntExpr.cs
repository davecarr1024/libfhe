using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class IntExpr : Expr
    {
        public int Value { get; set; }

        public IntExpr(int value)
        {
            Value = value;
        }

        public Val Eval(Scope scope)
        {
            return new IntVal(Value);
        }
    }
}
