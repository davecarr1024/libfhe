using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class NoneType : Object
    {
        public override string ToString()
        {
            return "None";
        }

        public override bool Equals(object obj)
        {
            return obj is NoneType;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [Attrs.BuiltinFunc]
        public NoneType()
            : base(BuiltinClass.Bind(typeof(NoneType)))
        {
        }

        [Attrs.BuiltinFunc]
        public Bool __eq__(NoneType rhs)
        {
            return new Bool(true);
        }
    }
}
