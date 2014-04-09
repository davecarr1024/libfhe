using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Float : Object
    {
        public float Value { get; set; }

        public Float(float value)
            : base(Class.GetBuiltinClass(typeof(Float)))
        {
            Value = value;
        }

        public override Val Clone()
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
