using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public class Sig
    {
        public Val RetType { get; private set; }

        public List<Param> Params { get; private set; }

        public Sig(Val retType, params Param[] paramList)
        {
            RetType = retType;
            Params = paramList.ToList();
        }
    }
}
