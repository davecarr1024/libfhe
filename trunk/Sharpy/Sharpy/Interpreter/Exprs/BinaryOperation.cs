using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class BinaryOperation : Expr
    {
        public enum Ops
        {
            Add,
            Sub,
            Mul,
            Div,
            IAdd,
            ISub,
            IMul,
            IDiv,
            Assign,
            Equal,
            NotEqual,
            Lt,
            Lte,
            Gt,
            Gte,
        }

        public Vals.Val Eval(Scope scope)
        {
            throw new NotImplementedException();
        }
    }
}
