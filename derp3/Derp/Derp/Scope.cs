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
                { "None", new Vals.None() },
                { "False", new Vals.Bool(false) },
                { "True", new Vals.Bool(true) },
                { "Assign", new Vals.BuiltinFunc(Derp.Assign) },
                { "Add", new Vals.BuiltinFunc(Derp.Add) },
                { "Subtract", new Vals.BuiltinFunc(Derp.Subtract) },
                { "Multiply", new Vals.BuiltinFunc(Derp.Multiply) },
                { "Divide", new Vals.BuiltinFunc(Derp.Divide) },
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
