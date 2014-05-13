using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class Bool : Object
    {
        public bool Value { get; private set; }

        public Bool(bool value)
            : base(BuiltinClass.Bind(typeof(Bool)))
        {
            Value = value;
        }

        public override string ToString()
        {
            return Value.ToString();
        }

        public override bool Equals(object obj)
        {
            return obj is Bool && (obj as Bool).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [Attrs.BuiltinFunc]
        public Bool()
            : this(false)
        {
        }

        [Attrs.BuiltinFunc]
        public Bool(Bool value)
            : this(value.Value)
        {
        }

        [Attrs.BuiltinFunc]
        public Bool __not__()
        {
            return new Bool(!Value);
        }
    }
}
