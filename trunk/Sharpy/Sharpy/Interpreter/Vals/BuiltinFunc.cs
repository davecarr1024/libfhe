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

        private Attrs.BuiltinFunc attr;

        public BuiltinFunc(MethodInfo method, Val obj)
        {
            Method = method;
            Obj = obj;
            attr = Method.GetCustomAttributes().OfType<Attrs.BuiltinFunc>().FirstOrDefault();
        }

        public override bool CanApply(params Val[] args)
        {
            return CanMethodApply(Method, args);
        }

        public override Val Apply(params Val[] argTypes)
        {
            try
            {
                return ConvertRet(Method.Invoke(Obj, argTypes));
            }
            catch (TargetInvocationException ex)
            {
                throw ex.InnerException;
            }
        }

        public static bool CanMethodApply(MethodBase method, params Val[] args)
        {
            return
                method.GetParameters().Length == args.Length &&
                Enumerable.Range(0, args.Length).All(i => method.GetParameters()[i].ParameterType.IsAssignableFrom(args[i].GetType()));
        }

        public override string ToString()
        {
            return Method.Name;
        }

        private Val ConvertRet(object ret)
        {
            return ret is Val ? ret as Val : new NoneType();
        }
    }
}
