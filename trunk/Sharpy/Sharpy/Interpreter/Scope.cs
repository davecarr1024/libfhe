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
            List<Vals.Val> argTypes = args.Select(arg => arg.Type).ToList();
            List<Var> vars = GetAll(name).Where(var => var.Val.CanApply(argTypes)).ToList();
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

        public bool CanSystemApply(string name)
        {
            return GetAll(name).Where(var => var.Val.CanSystemApply()).Count() == 1;
        }

        public Vals.Val SystemApply(string name, List<Exprs.Expr> exprs, Scope scope)
        {
            Vals.Val ret;
            if (TrySystemApply(name, exprs, scope, out ret))
            {
                return ret;
            }
            else
            {
                throw new Exception("failed system apply " + name);
            }
        }

        private bool TrySystemApply(string name, List<Exprs.Expr> exprs, Scope scope, out Vals.Val ret)
        {
            List<Var> vars = GetAll(name).Where(var => var.Val.CanSystemApply()).ToList();
            if ( vars.Count > 1 )
            {
                throw new Exception("ambiguous system call " + name);
            }
            else if ( vars.Count == 1 )
            {
                ret = vars.First().Val.SystemApply(exprs,scope);
                return true;
            }
            else if ( Parent != null )
            {
                return Parent.TrySystemApply(name,exprs,scope,out ret);
            }
            else
            {
                ret = null;
                return false;
            }
        }

        public Vals.Val Apply(string name, List<Exprs.Expr> exprs, Scope scope)
        {
            Vals.Val ret;
            if (TrySystemApply(name, exprs, scope, out ret))
            {
                return ret;
            }
            else
            {
                return Apply(name, exprs.Select(expr => expr.Eval(scope)).ToList());
            }
        }
    }
}
