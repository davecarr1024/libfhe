using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sharpy.Interpreter.Exprs
{
    public class UnboundBuiltinFunc : Expr
    {
        public MethodInfo Method { get; private set; }

        public UnboundBuiltinFunc(MethodInfo method)
        {
            Method = method;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val val = new Vals.BuiltinFunc(Method, scope.Get("this"));
            scope.Add(Method.Name, val);
            return val;
        }
    }
}
