using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public interface Val
    {
        Val Clone();

        Val Apply(List<Expr> args, Scope scope);
    }
}
