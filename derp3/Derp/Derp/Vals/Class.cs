using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Class : ScopeVal
    {
        public string Name { get; set; }

        public Scope Scope { get; set; }

        private static Dictionary<Type, Class> BuiltinClasses = new Dictionary<Type, Class>();

        private static Dictionary<Class, Type> BuiltinTypes = new Dictionary<Class, Type>();

        public static Class Bind(Type type)
        {
            if (!BuiltinClasses.ContainsKey(type))
            {
                BuiltinTypes[BuiltinClasses[type] = new Class(type)] = type;
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
            if (type.BaseType != null)
            {
                Scope.Parent = Bind(type.BaseType).Scope;
            }
            foreach (MethodInfo method in type.GetMethods().Where(m => m.GetCustomAttributes().Any(attr => attr is BuiltinFunc)))
            {
                Scope[method.Name] = new Builtin((args) =>
                    {
                        if (method.IsStatic)
                        {
                            return method.Invoke(null, args.Cast<object>().ToArray()) as Val;
                        }
                        else
                        {
                            return method.Invoke(args.First(), args.Skip(1).Cast<object>().ToArray()) as Val;
                        }
                    });
            }
        }

        public Val Clone()
        {
            throw new NotImplementedException();
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            if (Scope.ContainsKey("__new__"))
            {
                Object obj = Scope["__new__"].Apply(args, scope) as Object;
                if (obj == null)
                {
                    throw new Exception(Name + ".__new__() must return object");
                }
                return obj;
            }
            else
            {
                Object obj = new Object(this);
                if (obj.Scope.ContainsKey("__init__"))
                {
                    obj.Scope["__init__"].Apply(args, scope);
                }
                return obj;
            }
        }

        public bool AsBool()
        {
            return false;
        }
    }
}
