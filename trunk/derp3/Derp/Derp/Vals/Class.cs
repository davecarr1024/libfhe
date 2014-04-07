using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class Class : Val
    {
        public string Name { get; set; }

        public Scope Scope { get; set; }

        public Class(string name, Scope scope)
        {
            Name = name;
            Scope = scope;
        }

        public Val Clone()
        {
            return new Class(Name, Scope.Clone());
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            return new Object(this);
        }
    }
}
