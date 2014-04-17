using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class Bool : Object
    {
        public bool IsReturn { get; set; }

        public bool Value { get; private set; }

        public Bool(bool value)
            : base(Class.Bind(typeof(Bool)))
        {
            Value = value;
            IsReturn = false;
        }

        public override bool ToBool()
        {
            return Value;
        }

        public override bool Equals(object obj)
        {
            return obj is Bool && (obj as Bool).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        public override string ToString()
        {
            return Value ? "True" : "False";
        }

        [Attrs.BuiltinMethod]
        public static Bool __new__(Bool value)
        {
            return new Bool(value.Value);
        }

        [Attrs.BuiltinMethod]
        public static Bool __new__()
        {
            return new Bool(false);
        }
    }
}
