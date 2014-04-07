using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
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
    }
}
