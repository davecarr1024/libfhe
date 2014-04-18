using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class Class : Expr
    {
        public string Name { get; private set; }

        public List<Expr> Body { get; private set; }

        public Class(string name, List<Expr> body)
        {
            Name = name;
            Body = body;
        }

        public Vals.Val Eval(Scope scope)
        {
            return scope[Name] = new Vals.Class(Name, Body, scope, null);
        }
    }
}
