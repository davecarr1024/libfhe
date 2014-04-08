using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Float : Val
    {
        public float Value { get; set; }

        public Float(float value)
        {
            Value = value;
        }

        public override Scope Clone()
        {
            return new Float(Value);
        }

        public override bool Equals(object obj)
        {
            return obj is Float && (obj as Float).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
