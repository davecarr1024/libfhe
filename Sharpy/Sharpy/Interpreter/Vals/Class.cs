using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public class Class : Val
    {
        public Val Type { get { return BuiltinClass.Bind(GetType()); } }

        public string Name { get; private set; }

        public Scope Scope { get; private set; }

        public List<Exprs.Expr> Body { get; private set; }

        public bool IsReturn { get; set; }

        public Class(string name, Scope scope, List<Exprs.Expr> body)
        {
            Name = name;
            Scope = scope;
            Body = body;
            IsReturn = false;
        }

        public bool CanApply(params Val[] argTypes)
        {
            return true;
        }

        public Val Apply(params Val[] args)
        {
            Object obj = new Object(this);
            if (Interpreter.CanApply(obj, "__init__", args.Select(arg => arg.Type).ToArray()))
            {
                Interpreter.Apply(obj, "__init__", args);
            }
            return obj;
        }
    }
}
