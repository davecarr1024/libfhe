using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Attrs
{
    public class BuiltinClass : Attribute
    {
        public List<string> Names { get; private set; }

        public BuiltinClass(params string[] names)
        {
            Names = names.ToList();
        }
    }
}
