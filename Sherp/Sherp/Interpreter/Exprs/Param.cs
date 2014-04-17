using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class Param
    {
        public string Type { get; private set; }

        public string Name { get; private set; }

        public Param(string type, string name)
        {
            Type = type;
            Name = name;
        }
    }
}
