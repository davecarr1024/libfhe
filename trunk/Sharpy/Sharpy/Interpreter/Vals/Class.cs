using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public class Class : Val
    {
        public string Name { get; private set; }

        public override Scope Scope { get; protected set; }

        public override List<Exprs.Expr> Body { get; protected set; }

        public Class(string name, Scope scope, List<Exprs.Expr> body)
        {
            Name = name;
            Scope = scope;
            Body = body;
        }

        public override bool CanApply(params Val[] args)
        {
            return true;
        }

        public override Val Apply(params Val[] args)
        {
            Object obj = new Object(this);
            if (obj.CanApply("__init__", args))
            {
                obj.Apply("__init__", args);
            }
            return obj;
        }
    }
}
