﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public class Param
    {
        public Val Type { get; private set; }

        public string Name { get; private set; }

        public Param(Val type, string name)
        {
            Type = type;
            Name = name;
        }

        public override string ToString()
        {
            return "<Param Type=\"" + Type + "\" Name=\"" + Name + "\"/>";
        }
    }
}
