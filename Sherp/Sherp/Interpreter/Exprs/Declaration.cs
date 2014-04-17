using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class Declaration : Expr
    {
        public Expr Type { get; private set; }

        public string Id { get; private set; }

        public Expr Value { get; private set; }

        public Declaration(Expr type, string id, Expr value)
        {
            Type = type;
            Id = id;
            Value = value;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val value = Value.Eval(scope);
            Vals.Val type = Type.Eval(scope);
            if (value.Type != type)
            {
                throw new Exception("unable to cast " + Id + " from " + value.Type + " to " + Type);
            }
            else
            {
                return scope[Id] = value;
            }
        }
    }
}
