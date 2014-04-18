using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public interface ApplyVal : Val
    {
        List<List<Param>> ParamsList { get; }

        Val Apply(List<Exprs.Expr> args, Scope scope);
    }
}
