using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Vals
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

        [BuiltinFunc]
        public static Int __new__(Int value)
        {
            return new Int(value.Value);
        }

        [BuiltinFunc]
        public Bool __eq__(Int value)
        {
            return new Bool(Value == value.Value);
        }

        public override bool Equals(object obj)
        {
            return obj is Int && (obj as Int).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
