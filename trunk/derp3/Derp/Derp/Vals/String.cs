using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class String : Val
    {
        public string Value { get; set; }

        public String(string value)
        {
            Value = value;
        }

        public override Scope Clone()
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
