using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter
{
    public class Var
    {
        public Vals.Val Type { get; private set; }

        public string Name { get; private set; }

        public Vals.Val Val { get; set; }

        public Var(Vals.Val type, string name, Vals.Val val)
        {
            Type = type;
            Name = name;
            Val = val;
        }
    }
}
