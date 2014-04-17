using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter
{
    public class Scope
    {
        public List<string> Keys
        {
            get
            {
                List<string> keys = Vals.Keys.ToList();
                foreach (Scope parent in Parents)
                {
                    keys.AddRange(parent.Keys);
                }
                return keys;
            }
        }

        public List<Vals.Val> Values
        {
            get
            {
                List<Vals.Val> values = Vals.Values.ToList();
                foreach (Scope parent in Parents)
                {
                    values.AddRange(parent.Values);
                }
                return values;
            }
        }

        public List<Scope> Parents { get; set; }

        private Dictionary<string, Vals.Val> Vals = new Dictionary<string, Vals.Val>();

        public Scope(params Scope[] parents)
        {
            Parents = parents.ToList();
        }

        public bool ContainsKey(string id)
        {
            return Vals.ContainsKey(id) || Parents.Any(parent => parent.ContainsKey(id));
        }

        public Vals.Val Get(string id)
        {
            Vals.Val val;
            if (TryGetValue(id, out val))
            {
                return val;
            }
            else
            {
                throw new Exception("unknown id " + id);
            }
        }

        public void Set(string id, Vals.Val value)
        {
            IEnumerable<Scope> parents = Parents.Where(parent => parent.ContainsKey(id));
            Vals.Val val;
            if (parents.Count() >= 2)
            {
                throw new Exception("duplicate id " + id);
            }
            else if (parents.Count() == 1)
            {
                parents.First().Set(id, value);
            }
            else if (!Vals.TryGetValue(id, out val) || val.Type == value.Type)
            {
                Vals[id] = value;
            }
            else
            {
                throw new Exception("unable to cast " + value.Type + " to " + val.Type);
            }
        }

        public Vals.Val this[string id]
        {
            get { return Get(id); }
            set { Set(id, value); }
        }

        public void Add(string id, Vals.Val value)
        {
            Vals.Val val;
            if (!Vals.TryGetValue(id, out val) || val.Type == value.Type)
            {
                Vals[id] = value;
            }
            else
            {
                throw new Exception("unable to cast " + value.Type + " to " + val.Type);
            }
        }

        public bool TryGetValue(string id, out Vals.Val val)
        {
            if (Vals.TryGetValue(id, out val))
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

        public bool Search(string id, out Vals.Val val)
        {
            if (TryGetValue(id, out val))
            {
                return true;
            }
            else
            {
                foreach (Scope scope in Values.OfType<Sherp.Interpreter.Vals.ScopeVal>().Select(scopeVal => scopeVal.Scope))
                {
                    if (scope.Search(id, out val))
                    {
                        return true;
                    }
                }
                return false;
            }
        }
    }
}
