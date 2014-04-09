using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class Class : Val
    {
        public string Name { get; set; }

        public Class(string name)
        {
            Name = name;
        }

        public override Scope Clone()
        {
            return new Class(Name);
        }

        public override Val Apply(List<Expr> args, Scope scope)
        {
            Val obj = new Val();
            return obj;
        }
    }
}
