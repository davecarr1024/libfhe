using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class Scope
    {
        public Dictionary<string, Val> Vals { get; set; }

        public Scope()
        {
            Vals = new Dictionary<string, Val>()
            {
                { "None", new NoneVal() },
                { "False", new BoolVal(false) },
                { "True", new BoolVal(true) },
                { "Assign", new BuiltinFuncVal(Derp.Assign) },
                { "Add", new BuiltinFuncVal(Derp.Add) },
                { "Subtract", new BuiltinFuncVal(Derp.Subtract) },
                { "Multiply", new BuiltinFuncVal(Derp.Multiply) },
                { "Divide", new BuiltinFuncVal(Derp.Divide) },
            };
        }

        public Scope Clone()
        {
            Scope scope = new Scope();
            foreach (KeyValuePair<string, Val> val in Vals)
            {
                scope.Vals[val.Key] = val.Value.Clone();
            }
            return scope;
        }
    }
}
