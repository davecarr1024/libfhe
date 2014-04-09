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
            : base(Class.Bind(typeof(Bool)))
        {
            Value = value;
        }

        public override bool AsBool()
        {
            return Value;
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
        public static Bool __new__(Bool value)
        {
            return new Bool(value.Value);
        }

        [BuiltinFunc]
        public String __str__()
        {
            return new String(Value.ToString());
        }

        [BuiltinFunc]
        public Bool __eq__(Bool rhs)
        {
            return new Bool(Value == rhs.Value);
        }

        [BuiltinFunc]
        public Bool __not__()
        {
            return new Bool(!Value);
        }
    }
}
