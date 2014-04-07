using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class FloatVal : Val
    {
        public float Value { get; set; }

        public FloatVal(float value)
        {
            Value = value;
        }

        public Val Clone()
        {
            return new FloatVal(Value);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }

        public override bool Equals(object obj)
        {
            return obj is FloatVal && (obj as FloatVal).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
