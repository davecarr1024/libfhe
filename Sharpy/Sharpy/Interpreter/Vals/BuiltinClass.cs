using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sharpy.Interpreter.Vals
{
    public class BuiltinClass : Val
    {
        public static Dictionary<Type, BuiltinClass> BuiltinClasses = new Dictionary<Type, BuiltinClass>();

        public static BuiltinClass Bind(Type type)
        {
            BuiltinClass builtinClass;
            if (!BuiltinClasses.TryGetValue(type, out builtinClass))
            {
                builtinClass = BuiltinClasses[type] = new BuiltinClass();
                builtinClass.Init(type);
            }
            return builtinClass;
        }

        public string Name { get; private set; }

        public List<Exprs.Expr> Body { get; private set; }

        public Scope Scope { get; private set; }

        public Val Type { get { return BuiltinClass.Bind(GetType()); } }

        public bool IsReturn { get; set; }

        private List<ConstructorInfo> ctors;

        public BuiltinClass()
        {
            IsReturn = false;
        }

        private void Init(Type type)
        {
            Name = type.Name;
            Scope = new Scope(null);
            Body = new List<Exprs.Expr>();
            foreach (MethodInfo method in type.GetMethods().Where(m => m.GetCustomAttributes().OfType<Attrs.BuiltinFunc>().Any()))
            {
                if (method.IsStatic)
                {
                    Scope.Add(method.Name, new BuiltinFunc(method, null));
                }
                else
                {
                    Body.Add(new Exprs.UnboundBuiltinFunc(method));
                }
            }
            ctors = type.GetConstructors().Where(ctor => ctor.GetCustomAttributes().OfType<Attrs.BuiltinFunc>().Any()).ToList();
        }

        public bool CanApply(List<Val> args)
        {
            return ctors.Any(ctor => BuiltinFunc.CanMethodApply(ctor, args));
        }

        public Val Apply(List<Val> args)
        {
            IEnumerable<ConstructorInfo> ctors = this.ctors.Where(ctor => BuiltinFunc.CanMethodApply(ctor, args));
            if (ctors.Count() > 1)
            {
                throw new Exception("ambiguous ctor for type " + Name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
            else if (ctors.Count() < 1)
            {
                throw new Exception("unknown ctor for type " + Name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
            else
            {
                Val val = ctors.First().Invoke(args.Cast<object>().ToArray()) as Val;
                if (val == null)
                {
                    throw new Exception("non-val builtinClass");
                }
                else
                {
                    return val;
                }
            }
        }

        public override string ToString()
        {
            return Name;
        }
    }
}
