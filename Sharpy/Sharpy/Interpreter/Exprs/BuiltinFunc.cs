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

        public BuiltinFunc(MethodInfo method)
        {
            Method = method;
        }

        public override Vals.Val Eval(Scope scope)
        {
            Vals.Val val = new Vals.BuiltinFunc(Method, scope.Get("this"));
            scope.Add(Method.Name, val);
            return val;
        }
    }
}
