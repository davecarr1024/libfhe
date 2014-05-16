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

        public virtual Scope Scope { get { return null; } protected set { } }

        public virtual List<Exprs.Expr> Body { get { return null; } protected set { } }

        public virtual bool IsReturn { get; set; }

        public Val()
        {
            IsReturn = false;
        }

        public virtual bool CanApply(params Val[] args)
        {
            return false;
        }

        public virtual Val Apply(params Val[] args)
        {
            throw new NotImplementedException();
        }

        public bool CanApply(string name, params Val[] argTypes)
        {
            return Scope != null && Scope.CanApply(name, argTypes);
        }

        public Val Apply(string name, params Val[] args)
        {
            if (Scope == null)
            {
                throw new Exception("obj " + this + " of type " + Type + " can't apply func " + name + ": no scope");
            }
            else if (!Scope.CanApply(name, args))
            {
                throw new Exception("obj " + this + " of type " + Type + " can't apply func " + name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
            else
            {
                return Scope.Apply(name, args);
            }
        }


        public bool CanConvert(Val toType)
        {
            return Type == toType || toType.CanApply(Type);
        }

        public Val Convert(Val toType)
        {
            if (Type == toType)
            {
                return this;
            }
            else if (toType.CanApply(Type))
            {
                return toType.Apply(this);
            }
            else
            {
                throw new Exception("unable to convert val " + this + " to type " + toType);
            }
        }
    }
}
