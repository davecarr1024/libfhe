using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Vals
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

        [BuiltinFunc]
        public static String __new__(String value)
        {
            return new String(value.Value);
        }

        [BuiltinFunc]
        public Bool __eq__(String value)
        {
            return new Bool(Value == value.Value);
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
