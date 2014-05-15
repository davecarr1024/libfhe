using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class System : Object
    {
        public System()
            : base(BuiltinClass.Bind(typeof(System)))
        {
        }

        [Attrs.BuiltinFunc]
        public static void Assert(Bool val)
        {
            if (!val.Value)
            {
                throw new Exceptions.AssertException();
            }
        }
    }
}
