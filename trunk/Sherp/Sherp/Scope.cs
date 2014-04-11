using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp
{
    public class Scope
    {
        public List<string> Keys
        {
            get
            {
                List<string> keys = vals.Keys.ToList();
                foreach (Scope parent in Parents)
                {
                    keys.AddRange(parent.Keys);
                }
                return keys;
            }
        }

        public List<Scope> Parents { get; private set; }

        private Dictionary<string, Val> vals = new Dictionary<string, Val>();

        public Val this[string id]
        {
            get
            {
                Val val;
                if (vals.TryGetValue(id, out val))
                {
                    return val;
                }
                else
                {
                    foreach (Scope parent in Parents)
                    {
                        if (parent.TryGetValue(id, out val))
                        {
                            return val;
                        }
                    }
                    throw new Exception("unknown id " + id);
                }
            }
            set
            {
                foreach (Scope parent in Parents)
                {
                    if (parent.ContainsKey(id))
                    {
                        parent[id] = value;
                        return;
                    }
                }
                vals[id] = value;
            }
        }

        public Scope(params Scope[] parents)
        {
            Parents = parents.ToList();
        }

        public bool ContainsKey(string id)
        {
            return vals.ContainsKey(id) || Parents.Any(parent => parent.ContainsKey(id));
        }

        public bool TryGetValue(string id, out Val val)
        {
            if (vals.TryGetValue(id, out val))
            {
                return true;
            }
            else
            {
                foreach (Scope parent in Parents)
                {
                    if (parent.TryGetValue(id, out val))
                    {
                        return true;
                    }
                }
                return false;
            }
        }
    }
}
