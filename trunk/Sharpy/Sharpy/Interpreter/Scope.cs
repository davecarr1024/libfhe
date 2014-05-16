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

        public void Set(string name, Vals.Val val)
        {
            List<Var> vars = GetAll(name).Where(var => val.CanConvert(var.Type)).ToList();
            if (vars.Count == 1)
            {
                vars.First().Val = val.Convert(vars.First().Type);
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

        public void Add(string name, Vals.Val val)
        {
            Add(val.Type, name, val);
        }

        public void Add(Vals.Val type, string name, Vals.Val val)
        {
            Vars.Add(new Var(type, name, val));
        }

        public bool CanApply(string name, params Vals.Val[] argTypes)
        {
            return GetAll(name).Where(var => var.Val.CanApply(argTypes)).Count() == 1 || (Parent != null && Parent.CanApply(name, argTypes));
        }

        public Vals.Val Apply(string name, params Vals.Val[] args)
        {
            List<Var> vars = GetAll(name);
            if (vars.Count == 0)
            {
                throw new Exception("unknown ref " + name);
            }
            vars = GetAll(name).Where(var => var.Val.CanApply(args)).ToList();
            if (vars.Count == 1)
            {
                return vars.First().Val.Apply(args);
            }
            else if (vars.Count > 1)
            {
                throw new Exception("ambiguous call to " + name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()));
            }
            else if (Parent != null)
            {
                return Parent.Apply(name, args);
            }
            else
            {
                throw new Exception("unknown call to " + name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
        }
    }
}
