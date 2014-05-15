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

        bool IsReturn { get; set; }

        bool CanApply(params Val[] argTypes);

        Val Apply(params Val[] args);
    }
}
