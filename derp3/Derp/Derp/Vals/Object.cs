using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class Object : ScopeVal
    {
        public Class Class { get; set; }

        public Scope Scope { get; set; }

        public Object(Class parentClass)
        {
            Class = parentClass;
            Scope = new Scope(Class.Scope);
        }

        public Val Clone()
        {
            return new Object(Class);
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }
    }
}
