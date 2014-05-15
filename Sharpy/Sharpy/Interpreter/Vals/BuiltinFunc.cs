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

        private Attrs.BuiltinFunc attr;

        public BuiltinFunc(MethodInfo method, Val obj)
        {
            IsReturn = false;
            Method = method;
            Obj = obj;
            attr = Method.GetCustomAttributes().OfType<Attrs.BuiltinFunc>().FirstOrDefault();
        }

        public bool CanApply(params Val[] argTypes)
        {
            return CanMethodApply(Method, argTypes);
        }

        public Val Apply(params Val[] argTypes)
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

        public static bool CanMethodApply(MethodBase method, params Val[] argTypes)
        {
            return
                argTypes.All(argType => argType is BuiltinClass) &&
                method.GetParameters().Length == argTypes.Length &&
                Enumerable.Range(0, argTypes.Length).All(i => method.GetParameters()[i].ParameterType.IsAssignableFrom((argTypes[i] as BuiltinClass).BuiltinType));
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
