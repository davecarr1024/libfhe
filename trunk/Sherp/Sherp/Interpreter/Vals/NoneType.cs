using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    [Attrs.BuiltinClass]
    public class NoneType : Object
    {
        public NoneType()
            : base(Class.Bind(typeof(NoneType)))
        {
            IsReturn = false;
        }

        public override bool Equals(object obj)
        {
            return obj is NoneType;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        public override string ToString()
        {
            return "None";
        }

        [Attrs.BuiltinMethod]
        public static NoneType __new__()
        {
            return new NoneType();
        }

        [Attrs.BuiltinMethod]
        public Bool __eq__(Val value)
        {
            return new Bool(value is NoneType);
        }

        [Attrs.BuiltinMethod]
        public Bool __neq__(Val value)
        {
            return new Bool(!(value is NoneType));
        }
    }
}
