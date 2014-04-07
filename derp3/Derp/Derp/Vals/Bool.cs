using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class Bool : Val
    {
        public bool Value { get; set; }

        public Bool(bool value)
        {
            Value = value;
        }

        public Val Clone()
        {
            return new Bool(Value);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }

        public override bool Equals(object obj)
        {
            return obj is Bool && (obj as Bool).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
