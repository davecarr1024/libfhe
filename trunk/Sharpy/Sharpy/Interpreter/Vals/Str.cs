using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class Str : Object
    {
        public string Value { get; private set; }

        public Str(string value)
            : base(BuiltinClass.Bind(typeof(Str)))
        {
            Value = value;
        }

        public override string ToString()
        {
            return string.Format("\"{0}\"", Value);
        }

        public override bool Equals(object obj)
        {
            return obj is Str && (obj as Str).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [Attrs.BuiltinFunc]
        public Str()
            : this("")
        {
        }

        [Attrs.BuiltinFunc]
        public Str(Str value)
            : this(value.Value)
        {
        }

    }
}
