using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Bool : Object
    {
        public bool Value { get; set; }

        public Bool(bool value)
            : base(Class.GetBuiltinClass(typeof(Bool)))
        {
            Value = value;
        }

        public override Val Clone()
        {
            return new Bool(Value);
        }

        public override bool Equals(object obj)
        {
            return obj is Bool && (obj as Bool).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [BuiltinFunc]
        public String __str__()
        {
            return new String(Value.ToString());
        }
    }
}
