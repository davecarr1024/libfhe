using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Derp
{
    public class ObjectVal : Val
    {
        public ClassVal Class { get; set; }

        public Scope Scope { get; set; }

        public ObjectVal(ClassVal class_)
        {
            Class = class_;
            Scope = new Scope();
            foreach (KeyValuePair<string, Val> val in Class.Scope.Vals)
            {
                if (val.Value is FuncVal)
                {
                    Scope.Vals[val.Key] = new MethodVal(this, val.Value as FuncVal);
                }
                else
                {
                    Scope.Vals[val.Key] = val.Value.Clone();
                }
            }
        }

        public Val Clone()
        {
            return new ObjectVal(Class)
            {
                Scope = Scope.Clone(),
            };
        }

        public Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }
    }
}
