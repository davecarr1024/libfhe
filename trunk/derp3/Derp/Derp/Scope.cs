using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class Scope
    {
        public Scope Parent { get; set; }

        private Dictionary<string, Val> vals = new Dictionary<string, Val>();

        public Scope()
        {
        }

        public Scope(Scope parent)
        {
            Parent = parent;
        }

        public Val this[string id]
        {
            get
            {
                Val val;
                if (vals.TryGetValue(id, out val))
                {
                    return val;
                }
                else if (Parent != null)
                {
                    return Parent[id];
                }
                else
                {
                    throw new Exception("invalid id");
                }
            }
            set
            {
                vals[id] = value;
            }
        }

        public bool ContainsKey(string id)
        {
            return vals.ContainsKey(id) || (Parent != null && Parent.ContainsKey(id));
        }

        public bool TryGetValue(string id, out Val val)
        {
            return vals.TryGetValue(id, out val) || (Parent != null && Parent.TryGetValue(id, out val));
        }

        public void Copy(Scope scope)
        {
            foreach (KeyValuePair<string, Val> val in scope.vals)
            {
                vals[val.Key] = val.Value.Clone() as Val;
            }
            if (scope.Parent != null)
            {
                Copy(scope.Parent);
            }
        }

        public virtual Scope Clone()
        {
            Scope scope = new Scope();
            scope.Copy(this);
            return scope;
        }

        public List<string> Keys
        {
            get
            {
                List<string> keys = vals.Keys.ToList();
                if (Parent != null)
                {
                    keys.AddRange(Parent.Keys);
                }
                return keys;
            }
        }
    }
}
