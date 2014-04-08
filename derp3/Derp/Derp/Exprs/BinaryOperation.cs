﻿using System;
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
        }

        private static Dictionary<Operators, string> OperatorFuncs = new Dictionary<Operators, string>()
        {
            { Operators.Add, "__add__" },
            { Operators.Subtract, "__sub__" },
            { Operators.Multiply, "__mul__" },
            { Operators.Divide, "__div__" },
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

        public Val Eval(Scope scope)
        {
            if (Operator == Operators.Assign)
            {
                if (Arg1 is Ref)
                {
                    Scope funcScope = (Arg1 as Ref).Resolve(scope);
                    return funcScope.Vals[(Arg1 as Ref).Ids.Last()] = Arg2.Eval(scope);
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
                if (arg1.Vals.TryGetValue(OperatorFuncs[Operator], out op))
                {
                    return op.Apply(new List<Expr>() { Arg1, Arg2 }, scope);
                }
                else
                {
                    throw new Exception("lhs " + arg1 + " doesn't support operator " + Operator + " with func " + OperatorFuncs[Operator]);
                }
            }
        }
    }
}
