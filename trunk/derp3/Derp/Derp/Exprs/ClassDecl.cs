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
            Vals.Class obj = new Vals.Class(Name) { Parent = scope };
            foreach (Expr expr in Body)
            {
                expr.Eval(obj);
            }
            return scope[Name] = obj;
        }
    }
}
