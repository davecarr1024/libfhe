using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter
{
    public class Mods
    {
        public enum Perms
        {
            Private,
            Public,
        }

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
