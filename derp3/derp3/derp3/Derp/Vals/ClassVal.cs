using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public class ClassVal : Val
    {
        public string Name { get; set; }

        public Scope Scope { get; set; }

        public ClassVal(string name, Scope scope)
        {
            Name = name;
            Scope = scope;
        }

        public Val Clone()
        {
            return new ClassVal(Name, Scope.Clone());
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            return new ObjectVal(this);
        }
    }
}
