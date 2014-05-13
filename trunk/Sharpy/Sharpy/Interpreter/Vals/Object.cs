using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public class Object : Val
    {
        public Val Type { get; private set; }

        public Scope Scope
        {
            get { throw new NotImplementedException(); }
        }

        public bool CanApply(List<Val> args)
        {
            throw new NotImplementedException();
        }

        public Val Apply(List<Val> args)
        {
            throw new NotImplementedException();
        }
    }
}
