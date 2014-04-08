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

        public override Scope Clone()
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
        public static Val __add__(List<Val> args)
        {
            if (args.Count == 2 && args[0] is Int && args[1] is Int)
            {
                return new Int((args[0] as Int).Value + (args[1] as Int).Value);
            }
            else
            {
                throw new NotImplementedException();
            }
        }
    }
}
