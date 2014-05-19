using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Int : Expr
    {
        public int Val { get; private set; }

        public Int(int val)
        {
            Val = val;
        }

        public override Vals.Val Eval(Scope scope)
        {
            return new Vals.Int(Val);
        }

        public override string ToString()
        {
            return Val.ToString();
        }
    }
}
