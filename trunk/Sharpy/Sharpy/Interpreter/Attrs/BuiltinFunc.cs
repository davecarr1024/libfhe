using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Attrs
{
    public class BuiltinFunc : Attribute
    {
        public bool IsSystem { get; private set; }

        public BuiltinFunc(bool isSystem)
        {
            IsSystem = isSystem;
        }

        public BuiltinFunc()
            : this(false)
        {
        }
    }
}
