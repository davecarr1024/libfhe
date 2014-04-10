using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class IfStatement : Expr
    {
        public Expr Cond { get; set; }

        public List<Expr> Body { get; set; }

        public List<Expr> NegBody { get; set; }

        public IfStatement(Expr cond, List<Expr> body, List<Expr> negBody)
        {
            Cond = cond;
            Body = body;
            NegBody = negBody;
        }

        public Val Eval(Scope scope)
        {
            List<Expr> body;
            Scope funcScope = new Scope(scope);
            if (Cond.Eval(funcScope).AsBool())
            {
                body = Body;
            }
            else
            {
                body = NegBody;
            }
            foreach (Expr expr in body)
            {
                Val val = expr.Eval(funcScope);
                if (val.IsReturn)
                {
                    return val;
                }
            }
            return null;
        }
    }
}
