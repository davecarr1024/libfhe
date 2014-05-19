using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sharpy.Interpreter.Exprs
{
    public class BuiltinFunc : Expr
    {
        public MethodInfo Method { get; private set; }

        public Sig Sig { get { return Vals.BuiltinFunc.MethodToSig(Method); } }

        public BuiltinFunc(MethodInfo method)
        {
            Method = method;
        }

        public override Vals.Val Eval(Scope scope)
        {
            Vals.BuiltinFunc val = new Vals.BuiltinFunc(Method, scope.Get("this"));
            scope.AddOverload(Method.Name, val);
            return val;
        }
    }
}
