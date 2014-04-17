using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sherp.Interpreter.Vals
{
    public class Class : ScopeVal, ApplyVal
    {
        private static Dictionary<Type, Class> builtinClasses = new Dictionary<Type, Class>();

        public static Class Bind(Type type)
        {
            Class c;
            if (!builtinClasses.TryGetValue(type, out c))
            {
                c = builtinClasses[type] = new Class(type);
            }
            return c;
        }

        public Val Type { get { return Class.Bind(GetType()); } }

        public bool IsReturn { get; set; }

        public string Name { get; set; }

        public Scope Scope { get; set; }

        public Class(string name, List<Exprs.Expr> body, Scope scope)
        {
            IsReturn = false;
            Name = name;
            Scope = new Scope();
            foreach (Exprs.Expr expr in body)
            {
                expr.Eval(scope);
            }
        }

        private Class(Type type)
        {
            IsReturn = false;
            Name = type.Name;
            Scope = new Scope();
            Init(type);
        }

        private void Init(Type type)
        {
            if (type.BaseType != null)
            {
                Init(type.BaseType);
            }
            foreach (MethodInfo method in type.GetMethods().Where(m => m.GetCustomAttributes().Any(a => a is Attrs.BuiltinMethod)))
            {
                Scope[method.Name] = new BuiltinMethod(
                    (args, scope) =>
                    {
                        if (method.IsStatic)
                        {
                            return method.Invoke(null, args.Select(arg => arg.Eval(scope)).Cast<object>().ToArray()) as Val;
                        }
                        else
                        {
                            return method.Invoke(args.First().Eval(scope), args.Skip(1).Select(arg => arg.Eval(scope)).Cast<object>().ToArray()) as Val;
                        }
                    }
                );
            }
        }

        public bool ToBool()
        {
            return true;
        }

        public Val Apply(List<Exprs.Expr> args, Scope scope)
        {
            Val obj;
            Val func;
            if (Scope.TryGetValue("__new__", out func) && func is ApplyVal)
            {
                obj = (func as ApplyVal).Apply(args, scope);
            }
            else
            {
                obj = new Object(this);
            }
            if (obj is ScopeVal && (obj as ScopeVal).Scope.TryGetValue("__init__", out func) && func is ApplyVal)
            {
                (func as ApplyVal).Apply(args, scope);
            }
            return obj;
        }

        public override string ToString()
        {
            return string.Format("<Class Name=\"{0}\"/>", Name);
        }
    }
}
