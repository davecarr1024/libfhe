using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Derp
{
    public class Val : Scope
    {
        private static Dictionary<Type, Scope> Types = new Dictionary<Type, Scope>();

        public Val()
        {
            Bind(GetType());
        }

        internal void Bind(Type type)
        {
            if (!Types.ContainsKey(type))
            {
                if (type.BaseType != null)
                {
                    Bind(type.BaseType);
                }

                if (type.GetCustomAttributes().Any(attr => attr is BuiltinClass))
                {
                    foreach (MethodInfo methodInfo in type.GetMethods().Where(method => method.GetCustomAttributes().Any(attr => attr is BuiltinFunc)))
                    {
                        this[methodInfo.Name] = new Vals.Builtin((args) => methodInfo.Invoke(null, new object[] { args }) as Val);
                    }
                }

                Types[type] = new Scope();
                Types[type].Copy(this);
            }
            else
            {
                Copy(Types[type]);
            }
        }

        public override Scope Clone()
        {
            Val val = new Val();
            val.Copy(this);
            return val;
        }

        public virtual Val Apply(List<Expr> args, Scope scope)
        {
            throw new NotImplementedException();
        }
    }
}
