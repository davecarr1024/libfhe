using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Int : Val
    {
        public int Value { get; set; }

        public Int(int value)
        {
            Value = value;
        }

        public Val Clone()
        {
            return new Int(Value);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
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
        public Int __add__(Int rhs)
        {
            return new Int(Value + rhs.Value);
        }
    }
}
