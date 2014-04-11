using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Vals
{
    public class Object : ScopeVal, CallVal
    {
        public Scope Scope { get; private set; }

        public bool IsReturn { get; set; }

        public Object(Class parent)
        {
            IsReturn = false;
            Scope = new Scope(parent.Scope);
        }

        public bool ToBool(Scope scope)
        {
            throw new NotImplementedException();
        }

        public Val Call(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }
    }
}
