using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public abstract class Val
    {
        public virtual Val Type { get { return BuiltinClass.Bind(GetType()); } }

        public virtual List<Sig> Sigs { get { return null; } protected set { } }

        public virtual List<Sig> InterfaceSigs { get { return null; } protected set { } }

        public virtual Scope Scope { get { return null; } protected set { } }

        public virtual List<Exprs.Expr> Body { get { return null; } protected set { } }

        public virtual Val Apply(params Val[] args)
        {
            throw new NotImplementedException();
        }

        public bool CanApply(params Val[] args)
        {
            return Sigs != null && Sigs.Any(sig => sig != null && sig.CanApply(args));
        }

        public bool CanApply(string name, params Val[] args)
        {
            return Scope != null && Scope.CanApply(name, args);
        }

        public Val Apply(string name, params Val[] args)
        {
            return Scope.Apply(name, args);
        }

        public bool CanInterfaceApply(params Val[] args)
        {
            return InterfaceSigs != null && InterfaceSigs.Any(sig => sig != null && sig.CanApply(args));
        }

        public bool CanConvert(Val toType)
        {
            return Type == toType || toType.CanApply(this);
        }

        public Val Convert(Val toType)
        {
            if (Type == toType)
            {
                return this;
            }
            else if (toType.CanApply(this))
            {
                return toType.Apply(this);
            }
            else
            {
                throw new Exception("unable to convert " + Type + " to " + toType);
            }
        }
    }
}
