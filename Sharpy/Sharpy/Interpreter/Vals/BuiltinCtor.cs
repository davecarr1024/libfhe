using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sharpy.Interpreter.Vals
{
    public class BuiltinCtor : Val
    {
        public ConstructorInfo Ctor { get; private set; }

        public override List<Sig> Sigs { get { return new List<Sig>() { Exprs.BuiltinCtor.CtorToSig(Ctor) }; } }

        public BuiltinCtor(ConstructorInfo ctor)
        {
            Ctor = ctor;
        }

        public override Val Apply(params Val[] args)
        {
            return Ctor.Invoke(args.ToArray()) as Object;
        }
    }
}
