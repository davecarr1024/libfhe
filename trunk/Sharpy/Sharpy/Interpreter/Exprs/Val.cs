using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Val : Expr
    {
        public Vals.Val Val { get; private set; }

        public Val(Vals.Val val)
        {
            Val = val;
        }

        public Vals.Val Eval(Scope scope)
        {
            return Val;
        }
    }
}
