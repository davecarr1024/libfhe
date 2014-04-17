using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class BinaryOperation : Expr
    {
        public enum Operators
        {
            Assign,
            Equals,
            NotEquals,
            Add,
            Subtract,
            Multiply,
            Divide,
            LessThan,
            LessThanOrEqual,
            GreaterThan,
            GreaterThanOrEqual,
        }

        private static Dictionary<Operators, string> operatorFuncs = new Dictionary<Operators, string>()
        {
            { Operators.Equals, "__eq__" },
            { Operators.NotEquals, "__neq__" },
            { Operators.Add, "__add__" },
            { Operators.Subtract, "__sub__" },
            { Operators.Multiply, "__mul__" },
            { Operators.Divide, "__div__" },
            { Operators.LessThan, "__lt__" },
            { Operators.LessThanOrEqual, "__lte__" },
            { Operators.GreaterThan, "__gt__" },
            { Operators.GreaterThanOrEqual, "__gte__" },
        };

        private static Dictionary<string, Operators> strToOperator = new Dictionary<string, Operators>()
        {
            { "=", Operators.Assign },
            { "==", Operators.Equals },
            { "!=", Operators.NotEquals },
            { "+", Operators.Add },
            { "-", Operators.Subtract },
            { "*", Operators.Multiply },
            { "/", Operators.Divide },
            { "<", Operators.LessThan },
            { "<=", Operators.LessThanOrEqual },
            { ">", Operators.GreaterThan },
            { ">=", Operators.GreaterThanOrEqual },
        };

        private static Dictionary<Operators, string> operatorToStr = new Dictionary<Operators, string>()
        {
            { Operators.Assign, "=" },
            { Operators.Equals, "==" },
            { Operators.NotEquals, "!=" },
            { Operators.Add, "+" },
            { Operators.Subtract, "-" },
            { Operators.Multiply, "*" },
            { Operators.Divide, "/" },
            { Operators.LessThan, "<" },
            { Operators.LessThanOrEqual, "<=" },
            { Operators.GreaterThan, ">" },
            { Operators.GreaterThanOrEqual, ">=" },
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
                throw new Exception("unknown binary operator " + str);
            }
        }

        public Operators Operator { get; private set; }

        public Expr Arg1 { get; private set; }

        public Expr Arg2 { get; private set; }

        public BinaryOperation(Operators op, Expr arg1, Expr arg2)
        {
            Operator = op;
            Arg1 = arg1;
            Arg2 = arg2;
        }

        public Vals.Val Eval(Scope scope)
        {
            if (Operator == Operators.Assign)
            {
                if (Arg1 is Ref)
                {
                    return (Arg1 as Ref).Resolve(scope)[(Arg1 as Ref).Ids.Last()] = Arg2.Eval(scope);
                }
                else
                {
                    throw new Exception("lhs of assign must be ref");
                }
            }
            else
            {
                Vals.Val arg1 = Arg1.Eval(scope);
                if (arg1 is Vals.ScopeVal)
                {
                    Vals.Val func;
                    string funcName = operatorFuncs[Operator];
                    if ((arg1 as Vals.ScopeVal).Scope.TryGetValue(funcName, out func))
                    {
                        if (func is Vals.ApplyVal)
                        {
                            return (func as Vals.ApplyVal).Apply(new List<Expr>() { Arg2 }, scope);
                        }
                        else
                        {
                            throw new Exception("BinaryOperation lhs of type " + arg1.Type + " func " + func + " isn't callable");
                        }
                    }
                    else
                    {
                        throw new Exception("BinaryOperation lhs of type " + arg1.Type + " doesn't have func " + funcName);
                    }
                }
                else
                {
                    throw new Exception("lhs of binaryOperation must be scopeval");
                }
            }
        }

        public override string ToString()
        {
            return Arg1 + " " + operatorToStr[Operator] + " " + Arg2;
        }
    }
}
