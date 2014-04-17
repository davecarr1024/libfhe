using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public interface Val
    {
        Val Type { get; }

        bool IsReturn { get; set; }

        bool ToBool();
    }
}
