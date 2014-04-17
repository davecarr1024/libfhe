using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public class Namespace : ScopeVal
    {
        public Scope Scope { get; private set; }

        public bool IsReturn { get; set; }

        public Val Type { get { return Class.Bind(GetType()); } }

        public Namespace(Scope scope)
        {
            IsReturn = false;
            Scope = new Scope(scope);
        }

        public bool ToBool()
        {
            return true;
        }
    }
}
