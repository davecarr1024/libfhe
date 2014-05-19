using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass("bool")]
    public class Bool : Object
    {
        public bool Val { get; private set; }

        public Bool(bool val)
            : base(BuiltinClass.Bind(typeof(Bool)))
        {
            Val = val;
        }

        public override bool Equals(object obj)
        {
            return obj is Bool && (obj as Bool).Val == Val;
        }

        public override string ToString()
        {
            return Val ? "true" : "false";
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [Attrs.BuiltinFunc]
        public Bool(Bool val)
            : this(val.Val)
        {
        }

        [Attrs.BuiltinFunc]
        public Bool __eq__(Bool val)
        {
            return new Bool(Val == val.Val);
        }
    }
}
