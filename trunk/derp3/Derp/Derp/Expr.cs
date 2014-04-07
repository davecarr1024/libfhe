using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public interface Expr
    {
        Val Eval(Scope scope);
    }
}
