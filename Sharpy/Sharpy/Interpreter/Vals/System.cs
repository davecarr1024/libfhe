using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class System : Val
    {
        [Attrs.BuiltinFunc]
        public static void Assert(Bool val)
        {
            if (!val.Val)
            {
                throw new Exception("assertion failed");
            }
        }
    }
}
