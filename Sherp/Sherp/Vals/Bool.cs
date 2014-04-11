using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Vals
{
    [BuiltinClass]
    public class Bool : Object
    {
        public bool Value { get; set; }

        public Bool(bool value)
            : base(Class.Bind(typeof(Bool)))
        {
            Value = value;
        }

        [BuiltinFunc]
        public static Bool __new__(Bool value)
        {
            return new Bool(value.Value);
        }

        [BuiltinFunc]
        public Bool __eq__(Bool value)
        {
            return new Bool(Value == value.Value);
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
