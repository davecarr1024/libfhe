using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Return : Expr
    {
        public Expr Val { get; private set; }

        public Return(Expr val)
        {
            Val = val;
        }

        public override Vals.Val Eval(Scope scope)
        {
            Vals.Val val = Val.Eval(scope);
            val.IsReturn = true;
            return val;
        }

        public override string ToString()
        {
            return string.Format("return {0};", Val);
        }
    }
}
