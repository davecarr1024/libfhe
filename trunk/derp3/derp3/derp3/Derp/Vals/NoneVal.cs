using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public class NoneVal : Val
    {
        public Val Clone()
        {
            return new NoneVal();
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }

        public override bool Equals(object obj)
        {
            return obj is NoneVal;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
