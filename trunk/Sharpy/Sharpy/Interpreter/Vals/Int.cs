using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class Int : Object
    {
        public int Value { get; private set; }

        public Int(int value)
            :base(BuiltinClass.Bind(typeof(Int)))
        {
            Value = value;
        }

        public override string ToString()
        {
            return Value.ToString();
        }

        public override bool Equals(object obj)
        {
            return obj is Int && (obj as Int).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [Attrs.BuiltinFunc]
        public Int()
            : this(0)
        {
        }

        [Attrs.BuiltinFunc]
        public Int(Int value)
            : this(value.Value)
        {
        }

        [Attrs.BuiltinFunc]
        public Int __neg__()
        {
            return new Int(-Value);
        }

        [Attrs.BuiltinFunc]
        public Int __inc__()
        {
            ++Value;
            return new Int(Value);
        }

        [Attrs.BuiltinFunc]
        public Int __dec__()
        {
            --Value;
            return new Int(Value);
        }
    }
}
