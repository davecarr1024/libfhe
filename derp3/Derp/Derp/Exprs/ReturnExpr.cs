using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class ReturnExpr : Expr
    {
        public Expr Expr { get; set; }

        public ReturnExpr(Expr expr)
        {
            Expr = expr;
        }

        public Val Eval(Scope scope)
        {
            Val val = Expr.Eval(scope);
            val.IsReturn = true;
            return val;
        }
    }
}
