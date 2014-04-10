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
            : base(Class.Bind(typeof(NoneType)))
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

        public override bool AsBool()
        {
            return false;
        }

        [BuiltinFunc]
        public static NoneType __new__()
        {
            return new NoneType();
        }

        [BuiltinFunc]
        public Bool __eq__(Val rhs)
        {
            return new Bool(rhs is NoneType);
        }
    }
}
