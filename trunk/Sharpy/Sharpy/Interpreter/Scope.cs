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
            List<Var> vars = Vars.Where(var => var.Name == name).ToList();
            if (Parent != null)
            {
                vars.AddRange(Parent.GetAll(name));
            }
            return vars;
        }

        public Vals.Val Get(string name)
        {
            List<Var> vars = GetAll(name);
            if (vars.Count > 1)
            {
                throw new Exception("ambiguous ref " + name);
            }
            else if (vars.Count < 1)
            {
                throw new Exception("unknown ref " + name);
            }
            else
            {
                return vars.First().Val;
            }
        }

        public void Set(string name, Vals.Val val)
        {
            List<Var> vars = GetAll(name).Where(var => Interpreter.CanConvert(val.Type, var.Type)).ToList();
            if (vars.Count > 1)
            {
                throw new Exception("ambiguous ref " + name);
            }
            else if (vars.Count < 1)
            {
                throw new Exception("unknown ref " + name);
            }
            else
            {
                vars.First().Val = Interpreter.Convert(val, vars.First().Type);
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

        public bool CanApply(string name, params Vals.Val[] args)
        {
            return GetAll(name).Where(var => var.Val.CanApply(args.Select(arg => arg.Type).ToArray())).Count() == 1;
        }

        public Vals.Val Apply(string name, params Vals.Val[] args)
        {
            List<Var> vars = GetAll(name).Where(var => var.Val.CanApply(args.Select(arg => arg.Type).ToArray())).ToList();
            if (vars.Count > 1)
            {
                throw new Exception("ambiguous call to " + name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()));
            }
            else if (vars.Count < 1)
            {
                throw new Exception("unknown call to " + name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
            else
            {
                return vars.First().Val.Apply(args);
            }
        }
    }
}
