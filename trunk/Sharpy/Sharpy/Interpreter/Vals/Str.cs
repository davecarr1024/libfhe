using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass("str")]
    public class Str : Object
    {
        public string Val { get; private set; }

        public Str(string val)
            : base(BuiltinClass.Bind(typeof(Str)))
        {
            Val = val;
        }

        public override string ToString()
        {
            return string.Format("\"{0}\"", Val);
        }

        public override bool Equals(object obj)
        {
            return obj is Str && (obj as Str).Val == Val;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [Attrs.BuiltinFunc]
        public Str(Str val)
            : this(val.Val)
        {
        }

        [Attrs.BuiltinFunc]
        public Bool __eq__(Str val)
        {
            return new Bool(Val == val.Val);
        }
    }
}
