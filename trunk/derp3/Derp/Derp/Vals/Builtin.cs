using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class Builtin : Val
    {
        public Func<List<Val>, Val> Func { get; set; }

        public bool IsReturn { get; set; }

        public Builtin(Func<List<Val>, Val> func)
        {
            Func = func;
            IsReturn = false;
        }

        public Val Clone()
        {
            return new Builtin(Func);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            return Func(args.Select(arg => arg.Eval(scope)).ToList());
        }

        public bool AsBool()
        {
            return false;
        }
    }
}
