using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public interface Val
    {
        Val Type { get; }

        Scope Scope { get; }

        List<Exprs.Expr> Body { get; }

        bool CanApply(List<Val> args);

        Val Apply(List<Val> args);
    }
}
