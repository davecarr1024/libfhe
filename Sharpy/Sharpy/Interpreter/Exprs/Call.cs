using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Call : Expr
    {
        public Expr Obj { get; private set; }

        public List<Expr> Args { get; private set; }

        public Call(Expr obj, List<Expr> args)
        {
            Obj = obj;
            Args = args;
        }

        public Vals.Val Eval(Scope scope)
        {
            List<Vals.Val> args = Args.Select(arg => arg.Eval(scope)).ToList();
            if (Obj is Ref)
            {
                scope = (Obj as Ref).Resolve(scope);
                IEnumerable<Var> vars = scope.GetAll((Obj as Ref).Ids.Last()).Where(var => var.Val.CanApply(args));
                if (vars.Count() > 1)
                {
                    throw new Exception("ambiguous call");
                }
                else if (vars.Count() < 1)
                {
                    throw new Exception("invalid call");
                }
                else
                {
                    return vars.First().Val.Apply(args);
                }
            }
            else
            {
                Vals.Val obj = Obj.Eval(scope);
                if (obj.CanApply(args))
                {
                    return obj.Apply(args);
                }
                else
                {
                    throw new Exception("obj " + obj + " can't apply args " + string.Join(", ", args.Select(arg => arg.ToString())));
                }
            }
        }
    }
}
