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
                builtinClass = BuiltinClasses[type] = new BuiltinClass(type);
            }
            return builtinClass;
        }

        public List<Exprs.Expr> Body { get; private set; }

        public Scope Scope { get; private set; }

        public Val Type { get { return BuiltinClass.Bind(GetType()); } }

        public BuiltinClass(Type type)
        {
            Scope = new Scope(null);
            foreach (MethodInfo method in type.GetMethods())
            {
                Scope.Add(method.Name, new BuiltinFunc(method));
            }
        }

        public bool CanApply(List<Val> args)
        {
            throw new NotImplementedException();
        }

        public Val Apply(List<Val> args)
        {
            throw new NotImplementedException();
        }
    }
}
