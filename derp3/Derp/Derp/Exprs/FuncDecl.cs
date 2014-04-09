using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class FuncDecl : Expr
    {
        public string Name { get; set; }

        public List<string> Params { get; set; }

        public List<Expr> Body { get; set; }

        public FuncDecl(string name, List<string> paramList, List<Expr> body)
        {
            Name = name;
            Params = paramList;
            Body = body;
        }

        public Val Eval(Scope scope)
        {
            return scope[Name] = new Vals.Func(Params, Body);
        }
    }
}
