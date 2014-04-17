using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class Direct : Expr
    {
        public Vals.Val Val { get; private set; }

        public Direct(Vals.Val val)
        {
            Val = val;
        }

        public Vals.Val Eval(Scope scope)
        {
            return Val;
        }
    }
}
