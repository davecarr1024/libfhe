using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp
{
    public interface CallVal : Val
    {
        Val Call(List<Expr> args, Scope scope);
    }
}
