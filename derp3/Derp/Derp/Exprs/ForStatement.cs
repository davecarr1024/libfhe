using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class ForStatement : Expr
    {
        public Expr Init { get; set; }

        public Expr Cond { get; set; }

        public Expr Inc { get; set; }

        public List<Expr> Body { get; set; }

        public ForStatement(Expr init, Expr cond, Expr inc, List<Expr> body)
        {
            Init = init;
            Cond = cond;
            Inc = inc;
            Body = body;
        }

        public Val Eval(Scope scope)
        {
            Scope funcScope = new Scope(scope);
            Init.Eval(funcScope);
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
                Inc.Eval(funcScope);
            }
            return new Vals.NoneType();
        }
    }
}
