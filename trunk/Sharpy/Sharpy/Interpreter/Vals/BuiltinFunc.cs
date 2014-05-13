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

        public object Obj { get; set; }

        public BuiltinFunc(MethodInfo method)
        {
            Method = method;
            Obj = null;
        }

        public Val Type { get { return BuiltinClass.Bind(GetType()); } }

        public Scope Scope { get { return null; } }

        public bool CanApply(List<Val> args)
        {
            return Method.GetParameters().Length == args.Count &&
                Enumerable.Range(0, args.Count).All(i => Method.GetParameters()[i].ParameterType.IsAssignableFrom(args[i].GetType()));
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
                //return new NoneType();
                throw new NotImplementedException();
            }
        }
    }
}
