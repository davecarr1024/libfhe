using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter
{
    public class Scope
    {
        public Scope Parent { get; private set; }

        public List<Var> Vars { get; private set; }

        public Scope(Scope parent)
        {
            Parent = parent;
            Vars = new List<Var>();
        }

        public Scope()
            : this(null)
        {
        }

        public void AddOverload(Vals.Val type, string name, Vals.Val val)
        {
            Vars.Add(new Var(type, name, val));
        }

        public void AddOverload(string name, Vals.Val val)
        {
            AddOverload(val.Type, name, val);
        }

        public void Add(Vals.Val type, string name, Vals.Val val)
        {
            if (Vars.Any(var => var.Name == name))
            {
                throw new Exception("duplicate var " + name);
            }
            else
            {
                Vars.Add(new Var(type, name, val));
            }
        }

        public void Add(string name, Vals.Val val)
        {
            Add(val.Type, name, val);
        }

        public List<Var> GetAll(string name)
        {
            return Vars.Where(var => var.Name == name).ToList();
        }

        public Vals.Val Get(string name)
        {
            List<Var> vars = GetAll(name);
            if (vars.Count == 1)
            {
                return vars.First().Val;
            }
            else if (vars.Count > 1)
            {
                throw new Exception("ambiguous ref " + name);
            }
            else if (Parent != null)
            {
                return Parent.Get(name);
            }
            else
            {
                throw new Exception("unknown ref " + name);
            }
        }

        public bool Has(string name)
        {
            return GetAll(name).Count == 1;
        }

        public void Set(string name, Vals.Val val)
        {
            List<Var> vars = GetAll(name);
            if (vars.Count == 1)
            {
                if (val.CanConvert(vars.First().Type))
                {
                    vars.First().Val = val.Convert(vars.First().Type);
                }
                else
                {
                    throw new Exception("unable to convert " + val.Type + " to " + vars.First().Type);
                }
            }
            else if (vars.Count > 1)
            {
                throw new Exception("ambiguous ref " + name);
            }
            else if (Parent != null)
            {
                Parent.Set(name, val);
            }
            else
            {
                throw new Exception("unknown ref " + name);
            }
        }

        public bool CanApply(string name, params Vals.Val[] args)
        {
            return GetAll(name).Where(var => var.Val.CanApply(args)).Count() > 1 || (Parent != null && Parent.CanApply(name, args));
        }

        public Vals.Val Apply(string name, params Vals.Val[] args)
        {
            List<Var> vars = GetAll(name).Where(var => var.Val.CanApply(args)).ToList();
            if (vars.Count == 1)
            {
                return vars.First().Val.Apply(args);
            }
            else if (vars.Count > 1)
            {
                throw new Exception("ambiguous call " + name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
            else if (Parent != null)
            {
                return Parent.Apply(name, args);
            }
            else
            {
                throw new Exception("unknown call " + name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
        }
    }
}
