using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public class StringExpr : Expr
    {
        public string Value { get; set; }

        public StringExpr(string value)
        {
            Value = value;
        }

        public Val Eval(Scope scope)
        {
            return new StringVal(Value);
        }
    }
}
