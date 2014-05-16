using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Func : Expr
    {
        public override Mods Mods { get; protected set; }

        public string Name { get; private set; }

        public List<Param> Params { get; private set; }

        public Expr ReturnType { get; private set; }

        public List<Expr> Body { get; private set; }

        public Func(Mods mods, string name, List<Param> paramList, Expr returnType, List<Expr> body)
        {
            Mods = mods;
            Name = name;
            Params = paramList;
            ReturnType = returnType;
            Body = body;
        }

        public override Vals.Val Eval(Scope scope)
        {
            Vals.Func func = new Vals.Func(Name, Params.Select(p => p.Eval(scope)).ToList(), ReturnType.Eval(scope), Body, scope);
            scope.Add(func.Name, func);
            return func;
        }
    }
}
