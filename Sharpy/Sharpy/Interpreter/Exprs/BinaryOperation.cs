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
            And,
            Or,
        }

        public Ops Op { get; private set; }

        public Expr Lhs { get; private set; }

        public Expr Rhs { get; private set; }

        private static Dictionary<string, Ops> StrToOp = new Dictionary<string, Ops>()
        {
            { "+", Ops.Add },
            { "-", Ops.Sub },
            { "*", Ops.Mul },
            { "/", Ops.Div },
            { "+=", Ops.IAdd },
            { "-=", Ops.ISub },
            { "*=", Ops.IMul },
            { "/=", Ops.IDiv },
            { "=", Ops.Assign },
            { "==", Ops.Equal },
            { "!=", Ops.NotEqual },
            { "<", Ops.Lt },
            { "<=", Ops.Lte },
            { ">", Ops.Gt },
            { ">=", Ops.Gte },
            { "&&", Ops.And },
            { "||", Ops.Or },
        };

        private static Dictionary<Ops, string> OpToFunc = new Dictionary<Ops, string>()
        {
            { Ops.Add, "__add__" },
            { Ops.Sub, "__sub__" },
            { Ops.Mul, "__mul__" },
            { Ops.Div, "__div__" },
            { Ops.IAdd, "__iadd__" },
            { Ops.ISub, "__isub__" },
            { Ops.IMul, "__imul__" },
            { Ops.IDiv, "__idiv__" },
            { Ops.Equal, "__eq__" },
            { Ops.NotEqual, "__neq__" },
            { Ops.Lt, "__lt__" },
            { Ops.Lte, "__lte__" },
            { Ops.Gt, "__gt__" },
            { Ops.Gte, "__gte__" },
            { Ops.And, "__and__" },
            { Ops.Or, "__or__" },
        };

        private static Dictionary<Ops, string> OpToStr = new Dictionary<Ops, string>()
        {
            { Ops.Add, "+" },
            { Ops.Sub, "-" },
            { Ops.Mul, "*" },
            { Ops.Div, "/" },
            { Ops.IAdd, "+=" },
            { Ops.ISub, "-=" },
            { Ops.IMul, "*=" },
            { Ops.IDiv, "/=" },
            { Ops.Assign, "=" },
            { Ops.Equal, "==" },
            { Ops.NotEqual, "!=" },
            { Ops.Lt, "<" },
            { Ops.Lte, "<=" },
            { Ops.Gt, ">" },
            { Ops.Gte, ">=" },
            { Ops.And, "&&" },
            { Ops.Or, "||" },
        };

        private static Dictionary<Ops, Func<Vals.Val, Vals.Val, Vals.Val>> Fallbacks = new Dictionary<Ops, Func<Vals.Val, Vals.Val, Vals.Val>>()
        {
            { Ops.NotEqual, ( lhs, rhs ) => lhs.Apply("__eq__",rhs).Apply("__not__") },
            { Ops.Lte, ( lhs, rhs ) => lhs.Apply("__lt__",rhs).Apply("__or__",lhs.Apply("__eq__",rhs)) },
            { Ops.Gte, ( lhs, rhs ) => lhs.Apply("__gt__",rhs).Apply("__or__",lhs.Apply("__eq__",rhs)) },
        };

        public BinaryOperation(Expr lhs, string opStr, Expr rhs)
        {
            Lhs = lhs;
            Rhs = rhs;
            Ops op;
            if (StrToOp.TryGetValue(opStr, out op))
            {
                Op = op;
            }
            else
            {
                throw new Exception("invalid binary operator " + opStr);
            }
        }

        public override Vals.Val Eval(Scope scope)
        {
            if (Op == Ops.Assign)
            {
                if (Lhs is Exprs.Ref)
                {
                    Scope lhsScope = (Lhs as Ref).Resolve(scope);
                    Vals.Val rhs = Rhs.Eval(scope);
                    lhsScope.Set((Lhs as Ref).Ids.Last(), rhs);
                    return rhs;
                }
                else
                {
                    throw new Exception("lhs of assign must be ref");
                }
            }
            else
            {
                string func = OpToFunc[Op];
                Vals.Val lhs = Lhs.Eval(scope);
                Vals.Val rhs = Rhs.Eval(scope);
                Func<Vals.Val, Vals.Val, Vals.Val> fallback;
                if (lhs.CanApply(func, rhs))
                {
                    return lhs.Apply(func, rhs);
                }
                else if (Fallbacks.TryGetValue(Op, out fallback))
                {
                    return fallback(lhs, rhs);
                }
                else
                {
                    throw new Exception("binaryOp lhs of type " + lhs.Type + " doesn't support func " + func);
                }
            }
        }

        public override string ToString()
        {
            return string.Format("{0} {1} {2}", Lhs, OpToStr[Op], Rhs);
        }
    }
}
