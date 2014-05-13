using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter
{
    public class Scope
    {
        public List<Var> Vars { get; private set; }

        public Scope Parent { get; private set; }

        public Scope(Scope parent)
        {
            Parent = parent;
            Vars = new List<Var>();
        }

        public IEnumerable<Var> GetAll(string name)
        {
            return Vars.Where(var => var.Name == name);
        }

        public Vals.Val Get(string name)
        {
            IEnumerable<Var> vars = GetAll(name);
            if (vars.Count() > 1)
            {
                throw new Exception("ambiguous ref " + name);
            }
            else if (vars.Count() < 1)
            {
                throw new Exception("unknown ref " + name);
            }
            else if (Parent != null)
            {
                return Parent.Get(name);
            }
            else
            {
                return vars.First().Val;
            }
        }

        public void Set(string name, Vals.Val val)
        {
            IEnumerable<Var> vars = GetAll(name).Where(var => Interpreter.CanConvert(val.Type, var.Type));
            if (vars.Count() > 1)
            {
                throw new Exception("ambiguous ref " + name);
            }
            else if (vars.Count() < 1)
            {
                throw new Exception("unknown ref " + name);
            }
            else if (Parent != null)
            {
                Parent.Set(name, val);
            }
            else
            {
                vars.First().Val = val;
            }
        }

        public void Add(string name, Vals.Val val)
        {
            Add(val.Type, name, val);
        }

        public void Add(Vals.Val type, string name, Vals.Val val)
        {
            Vars.Add(new Var(type, name, val));
        }
    }
}
