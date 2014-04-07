using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class StringVal : Val
    {
        public string Value { get; set; }

        public StringVal(string value)
        {
            Value = value;
        }

        public Val Clone()
        {
            return new StringVal(Value);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }

        public override bool Equals(object obj)
        {
            return obj is StringVal && (obj as StringVal).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
