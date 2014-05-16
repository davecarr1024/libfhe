using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter
{
    public class Sig
    {
        public Vals.Val RetType { get; private set; }

        public List<Param> Params { get; private set; }

        public Sig(Vals.Val retType, params Param[] paramList)
        {
            RetType = retType;
            Params = paramList.ToList();
        }

        public bool CanApply(params Vals.Val[] args)
        {
            return
                Params.Count == args.Length &&
                Enumerable.Range(0, Params.Count).All(i => args[i].CanConvert(Params[i].Type));
        }
    }
}
