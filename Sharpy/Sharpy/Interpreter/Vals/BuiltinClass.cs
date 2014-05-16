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

        public override List<Sig> InterfaceSigs { get { return unboundFuncs.Select(func => func.Sig).ToList(); } }

        public override List<Sig> Sigs { get { return unboundFuncs.Where(func => func.Name == "__init__").Select(func => func.Sig).ToList(); } }

        private List<Exprs.BuiltinFunc> unboundFuncs = new List<Exprs.BuiltinFunc>();

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
                    Exprs.BuiltinFunc func = new Exprs.BuiltinFunc(method);
                    Body.Add(func);
                    unboundFuncs.Add(func);
                }
            }
        }

        public override Val Apply(params Val[] args)
        {
            throw new NotImplementedException();
        }
    }
}
