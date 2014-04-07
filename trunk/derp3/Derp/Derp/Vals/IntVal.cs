using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class IntVal : Val
    {
        public int Value { get; set; }

        public IntVal(int value)
        {
            Value = value;
        }

        public Val Clone()
        {
            return new IntVal(Value);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }

        public override bool Equals(object obj)
        {
            return obj is IntVal && (obj as IntVal).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
