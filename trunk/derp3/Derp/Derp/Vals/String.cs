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
            : base(Class.Bind(typeof(String)))
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

        [BuiltinFunc]
        public static String __new__(String value)
        {
            return new String(value.Value);
        }

        [BuiltinFunc]
        public Bool __eq__(String rhs)
        {
            return new Bool(Value == rhs.Value);
        }

        [BuiltinFunc]
        public String __add__(String rhs)
        {
            return new String(Value + rhs.Value);
        }

        [BuiltinFunc]
        public String __mul__(Int rhs)
        {
            return new String(string.Concat(Enumerable.Repeat(Value, rhs.Value)));
        }
    }
}
