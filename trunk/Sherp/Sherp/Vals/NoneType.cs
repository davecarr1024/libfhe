using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Vals
{
    [BuiltinClass]
    public class NoneType : Object
    {
        public NoneType()
            : base(Class.Bind(typeof(NoneType)))
        {
        }

        [BuiltinFunc]
        public static NoneType __new__()
        {
            return new NoneType();
        }

        [BuiltinFunc]
        public Bool __eq__(NoneType rhs)
        {
            return new Bool(true);
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
