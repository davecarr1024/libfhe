using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Exprs
{
    public class BinaryOperation : Expr
    {
        public enum Operators
        {
            Assign,
            Add,
            Subtract,
            Multiply,
            Divide,
            Equals,
            LessThanOrEqual,
            LessThan,
            GreaterThanOrEqual,
            GreaterThan,
        }

        private static Dictionary<Operators, string> OperatorFuncs = new Dictionary<Operators, string>()
        {
            { Operators.Add, "__add__" },
            { Operators.Subtract, "__sub__" },
            { Operators.Multiply, "__mul__" },
            { Operators.Divide, "__div__" },
            { Operators.Equals, "__eq__" },
            { Operators.LessThanOrEqual, "__lte__" },
            { Operators.LessThan, "__lt__" },
            { Operators.GreaterThanOrEqual, "__gte__" },
            { Operators.GreaterThan, "__gt__" },
        };

        private static Dictionary<Operators, string> OperatorSymbols = new Dictionary<Operators, string>()
        {
            { Operators.Add, "+" },
            { Operators.Subtract, "-" },
            { Operators.Multiply, "*" },
            { Operators.Divide, "/" },
            { Operators.Assign, "=" },
            { Operators.Equals, "==" },
            { Operators.LessThanOrEqual, "<=" },
            { Operators.LessThan, "<" },
            { Operators.GreaterThanOrEqual, ">=" },
            { Operators.GreaterThan, ">" },
        };

        public Operators Operator { get; set; }

        public Expr Arg1 { get; set; }

        public Expr Arg2 { get; set; }

        public BinaryOperation(Operators op, Expr arg1, Expr arg2)
        {
            Operator = op;
            Arg1 = arg1;
            Arg2 = arg2;
        }

        private Scope Resolve(Scope scope, string id)
        {
            if (scope.Parent != null && scope.Parent.ContainsKey(id))
            {
                return Resolve(scope.Parent, id);
            }
            else
            {
                return scope;
            }
        }

        public Val Eval(Scope scope)
        {
            if (Operator == Operators.Assign)
            {
                if (Arg1 is Ref)
                {
                    Scope funcScope = (Arg1 as Ref).Resolve(scope);
                    string id = (Arg1 as Ref).Ids.Last();
                    return Resolve(funcScope, id)[id] = Arg2.Eval(scope);
                }
                else
                {
                    throw new Exception("lhs of assign must be ref");
                }
            }
            else
            {
                Val arg1 = Arg1.Eval(scope);
                Val op;
                if (arg1 is ScopeVal && (arg1 as ScopeVal).Scope.TryGetValue(OperatorFuncs[Operator], out op))
                {
                    return op.Apply(new List<Expr>() { Arg2 }, scope);
                }
                else
                {
                    throw new Exception("lhs " + arg1 + " doesn't support operator " + Operator + " with func " + OperatorFuncs[Operator]);
                }
            }
        }

        public override string ToString()
        {
            return Arg1 + " " + OperatorSymbols[Operator] + " " + Arg2;
        }
    }
}
