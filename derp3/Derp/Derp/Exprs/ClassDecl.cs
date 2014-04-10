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

        public Expr Parent { get; set; }

        public ClassDecl(string name, List<Expr> body, Expr parent)
        {
            Name = name;
            Body = body;
            Parent = parent;
        }

        public Val Eval(Scope scope)
        {
            Vals.Class parent = null;
            if (Parent != null)
            {
                parent = Parent.Eval(scope) as Vals.Class;
                if (parent == null)
                {
                    throw new Exception("class " + Name + " can't inherit from non-class " + Parent);
                }
            }
            return scope[Name] = new Vals.Class(Name, Body, scope, parent);
        }

        public override string ToString()
        {
            return "class " + Name + "{" + string.Join("; ", Body.Select(expr => expr.ToString()).ToArray()) + "}";
        }
    }
}
