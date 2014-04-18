using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class UnaryOperation : Expr
    {
        public enum Operators
        {
            Negative,
            Not,
        }

        private static Dictionary<Operators, string> operatorFuncs = new Dictionary<Operators, string>()
        {
            { Operators.Negative, "__neq__" },
            { Operators.Not, "__not__" },
        };

        private static Dictionary<Operators, string> operatorToStr = new Dictionary<Operators, string>()
        {
            { Operators.Negative, "-" },
            { Operators.Not, "!" },
        };

        private static Dictionary<string, Operators> strToOperator = new Dictionary<string, Operators>()
        {
            { "-", Operators.Negative },
            { "!", Operators.Not },
        };

        public static Operators ParseOperator(string str)
        {
            Operators op;
            if (strToOperator.TryGetValue(str, out op))
            {
                return op;
            }
            else
            {
                throw new Exception("unknown unary operator " + str);
            }
        }

        public Operators Operator { get; private set; }

        public Expr Arg { get; private set; }

        public UnaryOperation(Operators op, Expr arg)
        {
            Operator = op;
            Arg = arg;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val arg = Arg.Eval(scope);
            if (arg is Vals.ScopeVal)
            {
                string funcName = operatorFuncs[Operator];
                Vals.Val func;
                if ((arg as Vals.ScopeVal).Scope.TryGetValue(funcName, out func))
                {
                    if (func is Vals.ApplyVal)
                    {
                        return (func as Vals.ApplyVal).Apply(new List<Expr>(), scope);
                    }
                    else
                    {
                        throw new Exception("UnaryOperation arg of type " + arg.Type + " func " + funcName + " isn't callable");
                    }
                }
                else
                {
                    throw new Exception("UnaryOperation arg of type " + arg.Type + " doesn't have func " + funcName);
                }
            }
            else
            {
                throw new Exception("UnaryOperation arg must be scopeVal");
            }
        }
    }
}
