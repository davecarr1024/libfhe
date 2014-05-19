using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Str : Expr
    {
        public string Val { get; private set; }

        public Str(string val)
        {
            Val = val;
        }

        public override Vals.Val Eval(Scope scope)
        {
            return new Vals.Str(Val);
        }

        public override string ToString()
        {
            return string.Format("\"{0}\"", Val);
        }
    }
}
