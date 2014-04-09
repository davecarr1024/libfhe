using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp.Vals
{
    [BuiltinClass]
    public class NoneType : Object
    {
        public NoneType()
            : base(Class.GetBuiltinClass(typeof(NoneType)))
        {
        }

        public override Val Clone()
        {
            return new NoneType();
        }

        public override bool Equals(object obj)
        {
            return obj is NoneType;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
