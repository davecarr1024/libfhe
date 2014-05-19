using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public class Decl : Expr
    {
        public Expr Type { get; private set; }

        public string Name { get; private set; }

        public Expr Val { get; private set; }

        public List<Expr> Args { get; private set; }

        private Decl(Expr type, string name, Expr val, List<Expr> args)
        {
            Type = type;
            Name = name;
            Val = val;
            Args = args;
        }

        public Decl(Expr type, string name)
            : this(type, name, null, null)
        {
        }

        public Decl(Expr type, string name, Expr val)
            : this(type, name, val, null)
        {
        }

        public Decl(Expr type, string name, List<Expr> args)
            : this(type, name, null, args)
        {
        }

        public override Vals.Val Eval(Scope scope)
        {
            Vals.Val type = Type.Eval(scope);
            Vals.Val val;
            if (Val != null)
            {
                val = Val.Eval(scope);
            }
            else if (Args != null)
            {
                val = type.Apply(Args.Select(arg => arg.Eval(scope)).ToArray());
            }
            else
            {
                val = type.Apply();
            }
            scope.Add(type, Name, val);
            return val;
        }

        public override string ToString()
        {
            if (Val != null)
            {
                return string.Format("{0} {1} = {2};", Type, Name, Val);
            }
            else if (Args != null)
            {
                return string.Format("{0} {1}({2});", Type, Name, string.Join(", ", Args.Select(arg => arg.ToString()).ToArray()));
            }
            else
            {
                return string.Format("{0} {1};", Type, Name);
            }
        }
    }
}
