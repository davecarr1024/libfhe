using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public abstract class Expr
    {
        public virtual Mods Mods { get { return new Mods(); } protected set { } }

        public virtual List<Sig> Sigs { get { return null; } protected set { } }

        public abstract Vals.Val Eval(Scope scope);

        public bool CanApply(params Vals.Val[] args)
        {
            return Sigs != null && Sigs.Any(sig => sig != null && sig.CanApply(args));
        }

        public bool CanApply(string name, params Vals.Val[] args)
        {
            return Sigs != null && Sigs.Any(sig => sig != null && sig.CanApply(name, args));
        }

        public static Expr Parse(Parser.Result result)
        {
            switch (result.Type)
            {
                case "statement":
                case "expr":
                case "decl":
                case "exprStatement":
                case "operand":
                    return Parse(result[0]);
                case "returnStatement":
                    return new Return(Parse(result[1]));
                case "ref":
                    return new Ref(result[0].Value, result[1].Children.Select(child => child[1].Value).ToArray());
                case "int":
                    return new Int(int.Parse(result.Value));
                case "str":
                    return new Str(result.Value.Substring(1, result.Value.Length - 2));
                case "call":
                    return new Call(Parse(result[0]), ParseArgList(result[1]));
                case "defDecl":
                    return new Decl(Parse(result[0]), result[1].Value);
                case "valDecl":
                    return new Decl(Parse(result[0]), result[1].Value, Parse(result[3]));
                case "argsDecl":
                    return new Decl(Parse(result[0]), result[1].Value, ParseArgList(result[2]));
                case "binaryOperation":
                    return new BinaryOperation(result[1][0].Value, Parse(result[0]), Parse(result[2]));
                default:
                    throw new NotImplementedException(result.Type);
            }
        }

        private static List<Expr> ParseArgList(Parser.Result result)
        {
            List<Expr> args = new List<Expr>();
            Parser.Result argResult = result[1].Children.FirstOrDefault();
            if (argResult != null)
            {
                args.Add(Parse(argResult[0]));
                args.AddRange(argResult[1].Children.Select(child => Parse(child[1])));
            }
            return args;
        }
    }
}
