using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Ctor : Expr
    {
        public Mods Mods { get; private set; }

        public string Name { get; private set; }

        public List<Param> Params { get; private set; }

        public List<Expr> Body { get; private set; }

        public Ctor(Mods mods, string name, List<Param> paramList, List<Expr> body)
        {
            Mods = mods;
            Name = name;
            Params = paramList;
            Body = body;
        }

        public Vals.Val Eval(Scope scope)
        {
            return new Func(Mods, "__init__", Params, new Ref("NoneType"), Body).Eval(scope);
        }

        public override string ToString()
        {
            return string.Format("{0}({1})", Name, string.Join(", ", Params.Select(p => p.ToString()).ToArray()));
        }
    }
}
