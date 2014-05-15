using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
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
            return string.Format("{0} {1}", Type, Name);
        }
    }
}
