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
            Assign,
            Equal,
        };

        public Ops Op { get; private set; }

        public Expr Lhs { get; private set; }

        public Expr Rhs { get; private set; }

        private static Dictionary<string, Ops> StrToOp = new Dictionary<string, Ops>()
        {
            { "=", Ops.Assign },
            { "==", Ops.Equal },
        };

        private static Dictionary<Ops, string> OpToStr = new Dictionary<Ops, string>()
        {
            {Ops.Assign,"="},
            {Ops.Equal,"=="},
        };

        private static Dictionary<Ops, string> OpToFunc = new Dictionary<Ops, string>()
        {
            {Ops.Equal,"__eq__"},
        };

        public BinaryOperation(string opStr, Expr lhs, Expr rhs)
        {
            Ops op;
            if (!StrToOp.TryGetValue(opStr, out op))
            {
                throw new Exception("invalid binary operator " + opStr);
            }
            Op = op;
            Lhs = lhs;
            Rhs = rhs;
        }

        public override Vals.Val Eval(Scope scope)
        {
            if (Op == Ops.Assign)
            {
                Ref lhs = Lhs as Ref;
                if (lhs == null)
                {
                    throw new Exception("lhs of assign must be ref");
                }
                else
                {
                    Vals.Val rhs = Rhs.Eval(scope);
                    lhs.Resolve(scope).Set(lhs.Names.Last(), rhs);
                    return rhs;
                }
            }
            else
            {
                Vals.Val lhs = Lhs.Eval(scope);
                Vals.Val rhs = Rhs.Eval(scope);
                string func = OpToFunc[Op];
                return lhs.Apply(func, rhs);
            }
        }

        public override string ToString()
        {
            return string.Format("{0} {1} {2}", Lhs, OpToStr[Op], Rhs);
        }
    }
}
