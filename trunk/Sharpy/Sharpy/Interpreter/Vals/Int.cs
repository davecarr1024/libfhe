using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass("int")]
    public class Int : Object
    {
        public int Val { get; private set; }

        public Int(int val)
            : base(BuiltinClass.Bind(typeof(Int)))
        {
            Val = val;
        }

        public override string ToString()
        {
            return Val.ToString();
        }

        public override bool Equals(object obj)
        {
            return obj is Int && (obj as Int).Val == Val;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [Attrs.BuiltinFunc]
        public Int(Int val)
            : this(val.Val)
        {
        }

        [Attrs.BuiltinFunc]
        public Bool __eq__(Int val)
        {
            return new Bool(Val == val.Val);
        }
    }
}
