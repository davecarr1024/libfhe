using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Class : ScopeVal
    {
        public string Name { get; set; }

        public Scope Scope { get; set; }

        public Class(string name, List<Expr> body, Scope parent)
        {
            Name = name;
            Scope = new Scope(parent);
            foreach (Expr expr in body)
            {
                expr.Eval(Scope);
            }
        }

        public Class(Type type, Scope parent)
        {
            Name = type.Name;
            Scope = new Scope(parent);
            foreach (MethodInfo method in type.GetMethods().Where(m => m.GetCustomAttributes().Any(attr => attr is BuiltinFunc)))
            {
                Scope[method.Name] = new Builtin((args) => method.Invoke(args.First(), args.Skip(1).Cast<object>().ToArray()) as Val);
            }
        }

        public Val Clone()
        {
            throw new NotImplementedException();
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }
    }
}
