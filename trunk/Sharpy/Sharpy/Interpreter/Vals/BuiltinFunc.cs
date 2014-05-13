using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sharpy.Interpreter.Vals
{
    public class BuiltinFunc : Val
    {
        public MethodInfo Method { get; private set; }

        public Val Obj { get; private set; }

        public List<Exprs.Expr> Body { get { return null; } }

        public Val Type { get { return BuiltinClass.Bind(GetType()); } }

        public Scope Scope { get { return null; } }

        public bool IsReturn { get; set; }

        public BuiltinFunc(MethodInfo method, Val obj)
        {
            IsReturn = false;
            Method = method;
            Obj = obj;
        }

        public bool CanApply(List<Val> args)
        {
            return CanMethodApply(Method, args);
        }

        public Val Apply(List<Val> args)
        {
            object ret = Method.Invoke(Obj, args.Cast<object>().ToArray());
            if (ret is Val)
            {
                return ret as Val;
            }
            else
            {
                return new NoneType();
            }
        }

        public static bool CanMethodApply(MethodBase method, List<Val> args)
        {
            return method.GetParameters().Length == args.Count &&
                Enumerable.Range(0, args.Count).All(i => method.GetParameters()[i].ParameterType.IsAssignableFrom(args[i].GetType()));
        }

        public override string ToString()
        {
            return Method.Name;
        }
    }
}
