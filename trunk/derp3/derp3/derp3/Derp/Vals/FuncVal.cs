﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public class FuncVal : Val
    {
        public List<string> Params { get; set; }

        public List<Expr> Body { get; set; }

        public FuncVal(List<string> paramList, List<Expr> body)
        {
            Params = paramList;
            Body = body;
        }

        public Val Clone()
        {
            return new FuncVal(Params, Body);
        }

        public Val Apply(List<Expr> args, Scope scope)
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
                return Body.Select(child => child.Eval(funcScope)).Last();
            }
        }
    }
}
