using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class Namespace : Expr
    {
        public string Name { get; set; }

        public Namespace(string name)
        {
            Name = name;
        }

        public Vals.Val Eval(Scope scope)
        {
            return scope[Name] = new Vals.Namespace(scope);
        }
    }
}
