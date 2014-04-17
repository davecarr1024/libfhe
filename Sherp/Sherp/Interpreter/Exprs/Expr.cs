using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public interface Expr
    {
        Vals.Val Eval(Scope scope);
    }
}
