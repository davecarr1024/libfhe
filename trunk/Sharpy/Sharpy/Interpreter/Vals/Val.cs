using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public abstract class Val
    {
        public virtual Val Type { get { return BuiltinClass.Bind(GetType()); } protected set { } }

        public virtual List<Sig> Sigs { get { return Type.InterfaceSigs; } protected set { } }

        public virtual List<Sig> InterfaceSigs
        {
            get
            {
                return Body != null ?
                    Body
                        .Where(expr => expr.Sigs != null)
                        .SelectMany(expr => expr.Sigs)
                        .Where(sig => sig != null)
                        .ToList()
                    : null;
            }
            protected set { }
        }

        public virtual Scope Scope { get { return null; } protected set { } }

        public virtual List<Exprs.Expr> Body { get { return null; } protected set { } }

        public bool IsReturn { get; set; }

        public Val()
        {
            IsReturn = false;
        }

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
            return Type.CanInterfaceApply(name, args);
        }

        public Val Apply(string name, params Val[] args)
        {
            if (Scope == null)
            {
                throw new Exception("trying to apply func " + name + " to non-scope val " + this);
            }
            else if (!Scope.Has(name))
            {
                throw new Exception("obj " + this + " of type " + Type + " has no member " + name);
            }
            else
            {
                Val func = Scope.Get(name);
                if (!func.CanApply(args))
                {
                    throw new Exception("unable to call member " + name + " of obj " + this + " of type " + Type + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
                }
                else
                {
                    return func.Apply(args);
                }
            }
        }

        public bool CanInterfaceApply(string name, params Val[] args)
        {
            return InterfaceSigs != null && InterfaceSigs.Any(sig => sig != null && sig.CanApply(name, args));
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
