using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class Int : Object
    {
        public int Value { get; private set; }

        public Int(int value)
            : base(Class.Bind(typeof(Int)))
        {
            Value = value;
        }

        [Attrs.BuiltinMethod]
        public static Int __new__(int value)
        {
            return new Int(value);
        }

        [Attrs.BuiltinMethod]
        public Bool __eq__(Int value)
        {
            return new Bool(Value == value.Value);
        }

        [Attrs.BuiltinMethod]
        public Bool __neq__(Int value)
        {
            return new Bool(Value != value.Value);
        }

        [Attrs.BuiltinMethod]
        public Int __add__(Int value)
        {
            return new Int(Value + value.Value);
        }

        [Attrs.BuiltinMethod]
        public Int __sub__(Int value)
        {
            return new Int(Value - value.Value);
        }

        [Attrs.BuiltinMethod]
        public Int __mul__(Int value)
        {
            return new Int(Value * value.Value);
        }

        [Attrs.BuiltinMethod]
        public Int __div__(Int value)
        {
            return new Int(Value / value.Value);
        }

        [Attrs.BuiltinMethod]
        public Bool __lt__(Int value)
        {
            return new Bool(Value < value.Value);
        }

        [Attrs.BuiltinMethod]
        public Bool __lte__(Int value)
        {
            return new Bool(Value <= value.Value);
        }

        [Attrs.BuiltinMethod]
        public Bool __gt__(Int value)
        {
            return new Bool(Value > value.Value);
        }

        [Attrs.BuiltinMethod]
        public Bool __gte__(Int value)
        {
            return new Bool(Value >= value.Value);
        }
    }
}
