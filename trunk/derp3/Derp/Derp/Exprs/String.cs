using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class String : Expr
    {
        public string Value { get; set; }

        public String(string value)
        {
            Value = value;
        }

        public Val Eval(Scope scope)
        {
            return new Vals.String(Value);
        }

        public override string ToString()
        {
            return Value.ToString();
        }
    }
}
