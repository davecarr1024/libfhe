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
                Class parent;
                if (type.BaseType != null)
                {
                    parent = Bind(type.BaseType);
                }
                else
                {
                    parent = null;
                }
                c = builtinClasses[type] = new Class(type.Name, parent);
                c.Init(type);
            }
            return c;
        }

        public Val Type { get { return Class.Bind(GetType()); } }

        public bool IsReturn { get; set; }

        public string Name { get; private set; }

        public Scope Scope { get; private set; }

        public Class Parent { get; private set; }

        public List<List<Param>> ParamsList
        {
            get
            {
                Val func;
                if (Scope.TryGetValue("__new__", out func) && func is ApplyVal)
                {
                    return (func as ApplyVal).ParamsList;
                }
                else if (Scope.TryGetValue("__init__", out func) && func is ApplyVal)
                {
                    return (func as ApplyVal).ParamsList;
                }
                else
                {
                    return new List<List<Param>>() { new List<Param>() };
                }
            }
        }

        public Class(string name, List<Exprs.Expr> body, Scope scope, Class parent)
        {
            IsReturn = false;
            Name = name;
            Parent = parent ?? Bind(typeof(Object));
            Scope = new Scope(Parent.Scope);
            foreach (Exprs.Expr expr in body)
            {
                expr.Eval(scope);
            }
        }

        private Class(string name, Class parent)
        {
            IsReturn = false;
            Name = name;
            Parent = parent;
            if (Parent != null)
            {
                Scope = new Scope(Parent.Scope);
            }
            else
            {
                Scope = new Scope();
            }
        }

        private void Init(Type type)
        {
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
                    },
                    GetParams(method)
                );
            }
            foreach (MethodInfo method in type.GetMethods())
            {
                Attrs.SystemMethod attr = method.GetCustomAttribute<Attrs.SystemMethod>();
                if (attr != null)
                {
                    Scope[method.Name] = new BuiltinMethod(
                        (args, scope) =>
                        {
                            return method.Invoke(null, new object[] { args, scope }) as Val;
                        },
                        attr.ArgTypes.Select(argType => new Param(Class.Bind(argType), "arg")).ToList()
                    );
                }
            }
        }

        private List<Param> GetParams(MethodInfo method)
        {
            List<Param> paramList = method.GetParameters().Select(param => new Param(Class.Bind(param.ParameterType), param.Name)).ToList();
            return paramList;
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
            return Name;
        }
    }
}
