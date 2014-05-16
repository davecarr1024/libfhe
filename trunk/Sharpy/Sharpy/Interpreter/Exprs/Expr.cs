using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public abstract class Expr
    {
        public virtual Mods Mods { get { return new Mods(); } protected set { } }

        public abstract Vals.Val Eval(Scope scope);
    }
}
