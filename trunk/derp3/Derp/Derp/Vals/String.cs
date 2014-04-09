using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class String : Object
    {
        public string Value { get; set; }

        public String(string value)
            : base(Class.GetBuiltinClass(typeof(String)))
        {
            Value = value;
        }

        public override Val Clone()
        {
            return new String(Value);
        }

        public override bool Equals(object obj)
        {
            return obj is String && (obj as String).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
