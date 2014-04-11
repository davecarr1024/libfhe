using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp
{
    public interface ScopeVal : Val
    {
        Scope Scope { get; }
    }
}
