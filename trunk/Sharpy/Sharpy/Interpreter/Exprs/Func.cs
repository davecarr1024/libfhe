using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Func : Expr
    {
        public string Name { get; private set; }

        public List<Param> Params { get; private set; }

        public Expr ReturnType { get; private set; }

        public List<Expr> Body { get; private set; }

        public Func(string name, List<Param> paramList, Expr returnType, List<Expr> body)
        {
            Name = name;
            Params = paramList;
            ReturnType = returnType;
            Body = body;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Func func = new Vals.Func(Name, Params.Select(p => p.Eval(scope)).ToList(), ReturnType.Eval(scope), Body, scope);
            scope.Add(func.Name, func);
            return func;
        }
    }
}
