using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Attrs
{
    public class SystemMethod : Attribute
    {
        public List<Type> ArgTypes { get; private set; }

        public SystemMethod(Type[] types)
        {
            ArgTypes = types.ToList();
        }
    }
}
