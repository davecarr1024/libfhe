using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class WhileStatement : Expr
    {
        public Expr Cond { get; set; }

        public List<Expr> Body { get; set; }

        public WhileStatement(Expr cond, List<Expr> body)
        {
            Cond = cond;
            Body = body;
        }

        public Val Eval(Scope scope)
        {
            Scope funcScope = new Scope(scope);
            while (Cond.Eval(funcScope).AsBool())
            {
                foreach (Expr expr in Body)
                {
                    Val val = expr.Eval(funcScope);
                    if (val.IsReturn)
                    {
                        return val;
                    }
                }
            }
            return new Vals.NoneType();
        }
    }
}
