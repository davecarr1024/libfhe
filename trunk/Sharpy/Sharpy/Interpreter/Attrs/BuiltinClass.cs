using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Attrs
{
    public class BuiltinClass : Attribute
    {
        public string Name { get; private set; }

        public BuiltinClass(string name)
        {
            Name = name;
        }

        public BuiltinClass()
            : this(null)
        {
        }
    }
}
