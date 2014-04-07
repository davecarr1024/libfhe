﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    public class None : Val
    {
        public Val Clone()
        {
            return new None();
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }

        public override bool Equals(object obj)
        {
            return obj is None;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}