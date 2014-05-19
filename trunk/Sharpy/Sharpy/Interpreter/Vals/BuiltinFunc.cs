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

        public override List<Sig> Sigs { get { return new List<Sig>() { MethodToSig(Method) }; } }

        public static Sig MethodToSig(MethodInfo method)
        {
            return new Sig(method.Name, BuiltinClass.Bind(method.ReturnType), method.GetParameters().Select(param => new Param(BuiltinClass.Bind(param.ParameterType), param.Name)).ToArray());
        }

        public BuiltinFunc(MethodInfo method, Val obj)
        {
            Method = method;
            Obj = obj;
        }

        public BuiltinFunc(MethodInfo method)
            : this(method, null)
        {
        }

        public override Val Apply(params Val[] args)
        {
            object obj = Method.Invoke(Obj, args.ToArray());
            if (obj is Val)
            {
                return obj as Val;
            }
            else
            {
                return new NoneType();
            }
        }
    }
}
