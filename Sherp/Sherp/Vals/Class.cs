using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sherp.Vals
{
    public class Class : ScopeVal, CallVal
    {
        public string Name { get; private set; }

        public Scope Scope { get; private set; }

        public bool IsReturn { get; set; }

        private static Dictionary<Type, Class> BuiltinClasses = new Dictionary<Type, Class>();

        public static Class Bind(Type type)
        {
            if (!BuiltinClasses.ContainsKey(type))
            {
                BuiltinClasses[type] = new Class(type);
            }
            return BuiltinClasses[type];
        }

        public Class(string name, List<Expr> body, Scope scope)
        {
            IsReturn = false;
            Name = name;
            Scope = new Scope(scope);
            foreach (Expr expr in body)
            {
                expr.Eval(Scope);
            }
        }

        public Class(string name, List<Expr> body, Scope scope, Class parent)
        {
            IsReturn = false;
            Name = name;
            Scope = new Scope(scope, parent.Scope);
            foreach (Expr expr in body)
            {
                expr.Eval(Scope);
            }
        }

        public Class(Type type)
        {
            Scope = new Scope();
            Init(type);
        }

        private void Init(Type type)
        {
            if (type.BaseType != null)
            {
                Init(type.BaseType);
            }
            foreach (MethodInfo method in type.GetMethods().Where(m => m.GetCustomAttributes().Any(attr => attr is BuiltinFunc)))
            {
                Scope[method.Name] = new Builtin((args, scope) =>
                    {
                        List<Val> vals = args.Select(arg => arg.Eval(scope)).ToList();
                        if (method.IsStatic)
                        {
                            return method.Invoke(null, vals.Cast<object>().ToArray()) as Val;
                        }
                        else
                        {
                            return method.Invoke(vals.First(), vals.Skip(1).Cast<object>().ToArray()) as Val;
                        }
                    });
            }
        }

        public Val Call(string id, List<Expr> args, Scope scope)
        {
            Val val;
            if (Scope.TryGetValue(id, out val))
            {
                CallVal callVal = val as CallVal;
                if (callVal != null)
                {
                    return callVal.Call(args, scope);
                }
                else
                {
                    throw new Exception("unable to call uncallable " + id);
                }
            }
            else
            {
                throw new Exception("unknown id " + id);
            }
        }

        public bool ToBool(Scope scope)
        {
            return Call("__bool__", new List<Expr>(), scope).ToBool(scope);
        }

        public Val Call(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }
    }
}
