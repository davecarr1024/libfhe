﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class ReturnStatement : Expr
    {
        public Expr Value { get; private set; }

        public ReturnStatement(Expr value)
        {
            Value = value;
        }

        public override Vals.Val Eval(Scope scope)
        {
            Vals.Val val = Value == null ? new Vals.NoneType() : Value.Eval(scope);
            val.IsReturn = true;
            return val;
        }
    }
}