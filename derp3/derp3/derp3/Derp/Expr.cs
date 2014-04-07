using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public interface Expr
    {
        Val Eval(Scope scope);
    }
}
