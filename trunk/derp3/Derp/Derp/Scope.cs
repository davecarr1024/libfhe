using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class Scope
    {
        public Dictionary<string, Val> Vals { get; set; }

        public Scope()
        {
            Vals = new Dictionary<string, Val>();
        }

        public void Copy(Scope scope)
        {
            foreach (KeyValuePair<string, Val> val in scope.Vals)
            {
                Vals[val.Key] = val.Value.Clone() as Val;
            }
        }

        public virtual Scope Clone()
        {
            Scope scope = new Scope();
            scope.Copy(this);
            return scope;
        }
    }
}
