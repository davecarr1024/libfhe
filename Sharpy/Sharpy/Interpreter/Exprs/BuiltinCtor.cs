using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sharpy.Interpreter.Exprs
{
    public class BuiltinCtor : Expr
    {
        public ConstructorInfo Ctor { get; private set; }

        public override List<Sig> Sigs { get { return new List<Sig>() { CtorToSig(Ctor) }; } }

        public static Sig CtorToSig(ConstructorInfo ctor)
        {
            return new Sig("__init__", null, ctor.GetParameters().Select(param => new Param(Vals.BuiltinClass.Bind(param.ParameterType), param.Name)).ToArray());
        }

        public BuiltinCtor(ConstructorInfo ctor)
        {
            Ctor = ctor;
        }

        public override Vals.Val Eval(Scope scope)
        {
            Vals.BuiltinCtor val = new Vals.BuiltinCtor(Ctor);
            scope.AddOverload("__init__", val);
            return val;
        }
    }
}
