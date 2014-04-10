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

        public bool IsReturn { get; set; }

        public Func(List<string> paramList, List<Expr> body)
        {
            IsReturn = false;
            Params = paramList;
            Body = body;
        }

        public Val Clone()
        {
            return new Func(Params, Body);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            if (args.Count != Params.Count)
            {
                throw new Exception("func arg count mismatch");
            }
            else
            {
                Scope funcScope = new Scope(scope);
                for (int i = 0; i < args.Count; ++i)
                {
                    funcScope[Params[i]] = args[i].Eval(scope);
                }
                foreach (Expr expr in Body)
                {
                    Val val = expr.Eval(funcScope);
                    if (val.IsReturn)
                    {
                        return val;
                    }
                }
                return new NoneType();
            }
        }

        public bool AsBool()
        {
            return false;
        }
    }
}
