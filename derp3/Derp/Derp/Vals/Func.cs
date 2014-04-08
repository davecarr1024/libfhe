using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class Func : Val
    {
        public List<string> Params { get; set; }

        public List<Expr> Body { get; set; }

        public Func(List<string> paramList, List<Expr> body)
        {
            Params = paramList;
            Body = body;
        }

        public override Val Apply(List<Expr> args, Scope scope)
        {
            if (args.Count != Params.Count)
            {
                throw new Exception("func arg count mismatch");
            }
            else
            {
                Scope funcScope = scope.Clone();
                for (int i = 0; i < args.Count; ++i)
                {
                    funcScope.Vals[Params[i]] = args[i].Eval(scope);
                }
                return Body.Select(body => body.Eval(funcScope)).Last();
            }
        }
    }
}
