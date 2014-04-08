using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Bool : Val
    {
        public bool Value { get; set; }

        public Bool(bool value)
        {
            Value = value;
        }

        public override Scope Clone()
        {
            return new Bool(Value);
        }

        public override bool Equals(object obj)
        {
            return obj is Bool && (obj as Bool).Value == Value;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [BuiltinFunc]
        public static Val __str__(List<Val> args)
        {
            if (args.Count == 1 && args[0] is Bool)
            {
                return new String((args[0] as Bool).Value.ToString());
            }
            else
            {
                throw new NotImplementedException();
            }
        }
    }
}
