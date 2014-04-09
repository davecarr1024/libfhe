using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class ClassDecl : Expr
    {
        public string Name { get; set; }

        public List<Expr> Body { get; set; }

        public ClassDecl(string name, List<Expr> body)
        {
            Name = name;
            Body = body;
        }

        public Val Eval(Scope scope)
        {
            return scope[Name] = new Vals.Class(Name, Body, scope);
        }
    }
}
