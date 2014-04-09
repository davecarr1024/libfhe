using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Int : Object
    {
        public int Value { get; set; }

        public Int(int value)
            : base(Class.Bind(typeof(Int)))
        {
            Value = value;
        }

        public override Val Clone()
        {
            return new Int(Value);
        }

        public override bool Equals(object obj)
        {
            return obj is Int && (obj as Int).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [BuiltinFunc]
        public static Int __new__(Int value)
        {
            return new Int(value.Value);
        }

        [BuiltinFunc]
        public Int __add__(Int rhs)
        {
            return new Int(Value + rhs.Value);
        }

        [BuiltinFunc]
        public Int __sub__(Int rhs)
        {
            return new Int(Value - rhs.Value);
        }

        [BuiltinFunc]
        public Int __mul__(Int rhs)
        {
            return new Int(Value * rhs.Value);
        }

        [BuiltinFunc]
        public Int __div__(Int rhs)
        {
            return new Int(Value / rhs.Value);
        }

        [BuiltinFunc]
        public Bool __eq__(Int rhs)
        {
            return new Bool(Value == rhs.Value);
        }

        [BuiltinFunc]
        public Int __neg__()
        {
            return new Int(-Value);
        }
    }
}
