using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Param
    {
        public Expr Type { get; private set; }

        public string Name { get; private set; }

        public Param(Expr type, string name)
        {
            Type = type;
            Name = name;
        }

        public Vals.Param Eval(Scope scope)
        {
            return new Vals.Param(Type.Eval(scope), Name);
        }

        public override string ToString()
        {
            return string.Format("{0} {1}", Type, Name);
        }
    }
}
