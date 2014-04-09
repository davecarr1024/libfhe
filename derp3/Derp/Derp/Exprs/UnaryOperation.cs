using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class UnaryOperation : Expr
    {
        public enum Operators
        {
            Not,
            Negative,
        }

        private static Dictionary<Operators, string> OperatorFuncs = new Dictionary<Operators, string>()
        {
            {Operators.Not,"__not__"},
            {Operators.Negative,"__neg__"},
        };

        private static Dictionary<Operators, string> OperatorSymbols = new Dictionary<Operators, string>()
        {
            {Operators.Not,"!"},
            {Operators.Negative,"-"},
        };

        public Operators Operator { get; set; }

        public Expr Arg { get; set; }

        public UnaryOperation(Operators op, Expr arg)
        {
            Operator = op;
            Arg = arg;
        }

        public Val Eval(Scope scope)
        {
            Val arg = Arg.Eval(scope);
            Val op;
            if (arg is ScopeVal && (arg as ScopeVal).Scope.TryGetValue(OperatorFuncs[Operator], out op))
            {
                return op.Apply(new List<Expr>(), scope);
            }
            else
            {
                throw new Exception(arg + " doesn't support operator " + Operator + " with func " + OperatorFuncs[Operator]);
            }
        }
    }
}
