using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter
{
    public class Param
    {
        public Vals.Val Type { get; private set; }

        public string Name { get; private set; }

        public Param(Vals.Val type, string name)
        {
            Type = type;
            Name = name;
        }
    }
}
