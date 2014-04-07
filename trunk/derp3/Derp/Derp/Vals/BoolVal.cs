using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class BoolVal : Val
    {
        public bool Value { get; set; }

        public BoolVal(bool value)
        {
            Value = value;
        }

        public Val Clone()
        {
            return new BoolVal(Value);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }

        public override bool Equals(object obj)
        {
            return obj is BoolVal && (obj as BoolVal).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
