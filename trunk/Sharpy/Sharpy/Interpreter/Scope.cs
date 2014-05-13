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
            List<Var> vars = GetAll(name).ToList();
            if (vars.Count > 1)
            {
                throw new Exception("ambiguous ref " + name);
            }
            else if (vars.Count == 1)
            {
                return vars.First().Val;
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
            List<Var> vars = GetAll(name).Where(var => Interpreter.CanConvert(val.Type, var.Type)).ToList();
            if (vars.Count > 1)
            {
                throw new Exception("ambiguous ref " + name);
            }
            else if (vars.Count == 1)
            {
                vars.First().Val = val;
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

        public bool CanApply(string name, List<Vals.Val> args)
        {
            return GetAll(name).Where(var => var.Val.CanApply(args)).Count() == 1;
        }

        public Vals.Val Apply(string name, List<Vals.Val> args)
        {
            List<Var> vars = GetAll(name).Where(var => var.Val.CanApply(args)).ToList();
            if (vars.Count > 1)
            {
                throw new Exception("ambiguous call to " + name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()));
            }
            else if (vars.Count == 1)
            {
                return vars.First().Val.Apply(args);
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
