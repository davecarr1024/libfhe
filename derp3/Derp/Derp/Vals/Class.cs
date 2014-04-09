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

        private static Dictionary<Type, Class> BuiltinClasses = new Dictionary<Type, Class>();

        public static Class GetBuiltinClass(Type type)
        {
            if (!BuiltinClasses.ContainsKey(type))
            {
                BuiltinClasses[type] = new Class(type);
            }
            return BuiltinClasses[type];
        }

        public Class(string name, List<Expr> body, Scope parent)
        {
            Name = name;
            Scope = new Scope(parent);
            foreach (Expr expr in body)
            {
                expr.Eval(Scope);
            }
        }

        private Class(Type type)
        {
            Name = type.Name;
            Scope = new Scope();
            Bind(type);
        }

        private void Bind(Type type)
        {
            if (type.BaseType != null)
            {
                Bind(type.BaseType);
            }
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
            return new Object(this);
        }
    }
}
