using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class UnaryOperation : Expr
    {
        public enum Operators
        {
            Not,
            Neg,
            Inc,
            Dec,
        }

        public Mods Mods { get { return new Mods(); } }

        public Operators Operator { get; private set; }

        public Expr Arg { get; private set; }

        private static Dictionary<Operators, string> OpToFunc = new Dictionary<Operators, string>()
        {
            { Operators.Neg, "__neg__" },
            { Operators.Not, "__not__" },
            { Operators.Inc, "__inc__" },
            { Operators.Dec, "__dec__" },
        };

        private static Dictionary<Operators, string> OpToStr = new Dictionary<Operators, string>()
        {
            { Operators.Neg, "-" },
            { Operators.Not, "!" },
            { Operators.Inc, "++" },
            { Operators.Dec, "--" },
        };

        private static Dictionary<string, Operators> StrToOp = new Dictionary<string, Operators>()
        {
            { "-", Operators.Neg },
            { "!", Operators.Not },
            { "++", Operators.Inc },
            { "--", Operators.Dec },
        };

        public UnaryOperation(string opStr, Expr arg)
        {
            Operators op;
            if (!StrToOp.TryGetValue(opStr, out op))
            {
                throw new Exception("invalid unary operator " + opStr);
            }
            else
            {
                Operator = op;
            }
            Arg = arg;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val arg = Arg.Eval(scope);
            string func = OpToFunc[Operator];
            if (arg.Scope == null || !arg.Scope.CanApply(func))
            {
                throw new Exception("unable to use operator " + OpToStr[Operator] + " on arg of type " + arg.Type + " with func " + func);
            }
            else
            {
                return arg.Scope.Apply(func);
            }
        }

        public override string ToString()
        {
            return OpToStr[Operator] + Arg;
        }
    }
}
