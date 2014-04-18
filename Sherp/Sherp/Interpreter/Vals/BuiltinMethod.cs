using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public class BuiltinMethod : ApplyVal
    {
        public Val Type { get { return Class.Bind(GetType()); } }

        public bool IsReturn { get; set; }

        public Func<List<Exprs.Expr>, Scope, Val> Func { get; private set; }

        public List<Param> Params { get; private set; }

        public List<List<Param>> ParamsList { get { return new List<List<Param>>() { Params }; } }

        public BuiltinMethod(Func<List<Exprs.Expr>, Scope, Val> func, List<Param> paramList)
        {
            IsReturn = false;
            Func = func;
            Params = paramList;
        }

        public Val Apply(List<Exprs.Expr> args, Scope scope)
        {
            List<Val> vals = args.Select(arg => arg.Eval(scope)).ToList();
            if (vals.Count != Params.Count || Enumerable.Range(0, vals.Count).Any(i => vals[i].Type != Params[i].Type))
            {
                throw new Exception("invalid args");
            }
            else
            {
                return Func(args, scope);
            }
        }

        public bool ToBool()
        {
            return true;
        }
    }
}
