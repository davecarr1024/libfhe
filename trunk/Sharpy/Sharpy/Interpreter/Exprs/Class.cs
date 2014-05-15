using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Class : Expr
    {
        public Mods Mods { get; private set; }

        public string Name { get; private set; }

        public List<Expr> Body { get; private set; }

        public Class(Mods mods, string name, List<Expr> body)
        {
            Mods = mods;
            Name = name;
            Body = body;
        }

        public Vals.Val Eval(Scope scope)
        {
            Scope classScope = new Scope(scope);
            List<Expr> classBody = new List<Expr>();
            foreach (Expr expr in Body)
            {
                if (expr is Ctor && (expr as Ctor).Name != Name)
                {
                    throw new Exception("mismatched ctor " + expr + " in class " + Name);
                }
                if (expr.Mods.Static)
                {
                    expr.Eval(classScope);
                }
                else
                {
                    classBody.Add(expr);
                }
            }
            Vals.Class c = new Vals.Class(Name, classScope, classBody);
            scope.Add(Name, c);
            return c;
        }
    }
}
