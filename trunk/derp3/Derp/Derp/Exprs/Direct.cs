using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class Direct : Expr
    {
        public Val Val { get; set; }

        public Direct(Val val)
        {
            Val = val;
        }

        public Val Eval(Scope scope)
        {
            return Val;
        }
    }
}
