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
        public bool IsReturn { get; set; }

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
        public static Bool __eq__(NoneType value)
        {
            return new Bool(true);
        }
    }
}
