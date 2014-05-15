using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Mods
    {
        public enum Perms
        {
            Public,
            Private,
        };

        public Perms Perm { get; private set; }

        public bool Static { get; private set; }

        public Mods(Perms perm, bool stat)
        {
            Perm = perm;
            Static = stat;
        }

        public Mods()
            : this(Perms.Private, false)
        {
        }
    }
}
