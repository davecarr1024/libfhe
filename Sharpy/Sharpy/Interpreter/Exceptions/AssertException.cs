﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exceptions
{
    public class AssertException : Exception
    {
        public AssertException(string msg)
            : base(msg)
        {
        }
    }
}
