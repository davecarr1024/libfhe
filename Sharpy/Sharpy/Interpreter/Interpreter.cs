using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter
{
    public static class Interpreter
    {
        public static bool CanConvert(Vals.Val fromType, Vals.Val toType)
        {
            return fromType == toType;
        }
    }
}
