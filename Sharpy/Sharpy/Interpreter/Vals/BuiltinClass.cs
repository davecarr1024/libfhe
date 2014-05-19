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
        public Type BuiltinType { get; private set; }

        public override Scope Scope { get; protected set; }

        public override List<Exprs.Expr> Body { get; protected set; }

        public override List<Sig> Sigs { get { return InterfaceSigs.Where(sig => sig.Name == "__init__").ToList(); } }

        private static Dictionary<Type, BuiltinClass> Builtins = new Dictionary<Type, BuiltinClass>();

        public static BuiltinClass Bind(Type type)
        {
            BuiltinClass builtin;
            if (!Builtins.TryGetValue(type, out builtin))
            {
                Builtins[type] = builtin = new BuiltinClass(type);
            }
            return builtin;
        }

        private BuiltinClass(Type type)
        {
            BuiltinType = type;
            Scope = new Scope();
            Body = new List<Exprs.Expr>();
            foreach (MethodInfo method in BuiltinType.GetMethods().Where(m => m.GetCustomAttributes().OfType<Attrs.BuiltinFunc>().Any()))
            {
                if (method.IsStatic)
                {
                    Scope.Add(method.Name, new BuiltinFunc(method));
                }
                else
                {
                    Body.Add(new Exprs.BuiltinFunc(method));
                }
            }
            foreach (ConstructorInfo ctor in BuiltinType.GetConstructors().Where(c => c.GetCustomAttributes().OfType<Attrs.BuiltinFunc>().Any()))
            {
                Body.Add(new Exprs.BuiltinCtor(ctor));
            }
        }

        public override Val Apply(params Val[] args)
        {
            List<Exprs.BuiltinCtor> ctors = Body.OfType<Exprs.BuiltinCtor>().Where(ctor => ctor.CanApply(args)).ToList();
            if (ctors.Count == 1)
            {
                return ctors.First().Eval(Scope).Apply(args);
            }
            else
            {
                throw new Exception("unable to construct builtin class " + this + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
        }

        public override string ToString()
        {
            return BuiltinType.Name;
        }
    }
}
