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

        public bool CanApply(List<Val> argTypes)
        {
            return CanMethodApply(Method, argTypes);
        }

        public Val Apply(List<Val> args)
        {
            try
            {
                return ConvertRet(Method.Invoke(Obj, args.ToArray()));
            }
            catch (TargetInvocationException ex)
            {
                throw ex.InnerException;
            }
        }

        public static bool CanMethodApply(MethodBase method, List<Val> argTypes)
        {
            return
                argTypes.All(argType => argType is BuiltinClass) &&
                method.GetParameters().Length == argTypes.Count &&
                Enumerable.Range(0, argTypes.Count).All(i => method.GetParameters()[i].ParameterType.IsAssignableFrom((argTypes[i] as BuiltinClass).BuiltinType));
        }

        public override string ToString()
        {
            return Method.Name;
        }

        public bool CanSystemApply()
        {
            return attr != null && attr.IsSystem;
        }

        public Val SystemApply(List<Exprs.Expr> exprs, Scope scope)
        {
            try
            {
                return ConvertRet(Method.Invoke(Obj, new object[] { exprs, scope }));
            }
            catch (TargetInvocationException ex)
            {
                throw ex.InnerException;
            }
        }

        private Val ConvertRet(object ret)
        {
            return ret is Val ? ret as Val : new NoneType();
        }
    }
}
