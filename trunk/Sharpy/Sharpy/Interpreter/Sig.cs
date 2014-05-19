using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter
{
    public class Sig
    {
        public string Name { get; private set; }

        public Vals.Val RetType { get; private set; }

        public List<Param> Params { get; private set; }

        public Sig(string name, Vals.Val retType, params Param[] paramList)
        {
            Name = name;
            RetType = retType;
            Params = paramList.ToList();
        }

        public bool CanApply(params Vals.Val[] args)
        {
            return
                Params.Count == args.Length &&
                Enumerable.Range(0, Params.Count).All(i => args[i].CanConvert(Params[i].Type));
        }

        public bool CanApply(string name, params Vals.Val[] args)
        {
            return
                Name == name &&
                Params.Count == args.Length &&
                Enumerable.Range(0, Params.Count).All(i => args[i].CanConvert(Params[i].Type));
        }
    }
}
