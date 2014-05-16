using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Exprs
{
    public abstract class Expr
    {
        public abstract Vals.Val Eval(Scope scope);

        public virtual Mods Mods { get { return new Mods(); } protected set { } }

        public static Expr Parse(Parser.Result result)
        {
            switch (result.Type)
            {
                case "statement":
                case "exprStatement":
                case "expr":
                case "operand":
                    return Parse(result[0]);
                case "parenExpr":
                    return Parse(result[1]);
                case "ref":
                    //ref => id ( '\.' id )*;
                    return new Ref(result[0].Value, result[1].Children.Select(child => child[1].Value).ToArray());
                case "int":
                    return new Int(int.Parse(result.Value));
                case "str":
                    return new Str(result.Value.Substring(1, result.Value.Length - 2));
                case "call":
                    //call => expr argList;
                    return new Call(Parse(result[0]), ParseArgs(result[1]));
                case "unaryOperation":
                    //unaryOperation => ( '\!' | '\-' ) expr;
                    return new UnaryOperation(result[0][0].Value, Parse(result[1]));
                case "varDecl":
                    //varDecl => mods id id ( '=' expr )? ';';
                    return new Decl(ParseMods(result[0]), Parse(result[1]), result[2].Value, result[3].Children.Any() ? Parse(result[3][0][1]) : null, null);
                case "varCtorDecl":
                    //varCtorDecl => mods ref id argList ';';
                    return new Decl(ParseMods(result[0]), Parse(result[1]), result[2].Value, null, ParseArgs(result[3]));
                case "binaryOperation":
                    //binaryOperation => operand ( '\+' | '\-' | '\*' | '\/' | '=' | '==' | '!=' | '<' | '<=' | '>' | '>=' | '\+=' | '\-=' | '\*=' | '\/=' ) operand;
                    return new BinaryOperation(Parse(result[0]), result[1][0].Value, Parse(result[2]));
                case "returnStatement":
                    //returnStatement => 'return' expr? ';';
                    return new ReturnStatement(result[1].Children.Any() ? Parse(result[1][0]) : null);
                case "funcDecl":
                    //funcDecl => mods retType id paramList body;
                    return new Func(ParseMods(result[0]), result[2].Value, ParseParams(result[3]), ParseRetType(result[1]), ParseBody(result[4]));
                case "classDecl":
                    //classDecl => mods 'class' id body;
                    return new Class(ParseMods(result[0]), result[2].Value, ParseBody(result[3]));
                case "ctor":
                    //ctor => mods id paramList body;
                    return new Ctor(ParseMods(result[0]), result[1].Value, ParseParams(result[2]), ParseBody(result[3]));
                default:
                    throw new NotImplementedException(result.Type);
            }
        }

        private static Mods ParseMods(Parser.Result result)
        {
            //mods => ( 'static' | 'private' | 'public' )*;
            Mods.Perms perm = Mods.Perms.Private;
            bool stat = false;
            foreach (string val in result.Children.Select(child => child[0].Value))
            {
                switch (val)
                {
                    case "static":
                        stat = true;
                        break;
                    case "private":
                        perm = Mods.Perms.Private;
                        break;
                    case "public":
                        perm = Mods.Perms.Public;
                        break;
                }
            }
            return new Mods(perm, stat);
        }

        private static List<Expr> ParseArgs(Parser.Result result)
        {
            //argList => '\(' ( expr ( ',' expr )* )? '\)';
            List<Expr> args = new List<Expr>();
            if (result[1].Children.Any())
            {
                Parser.Result argResult = result[1][0];
                args.Add(Parse(argResult[0]));
                args.AddRange(argResult[1].Children.Select(child => Parse(child[1])));
            }
            return args;
        }

        private static List<Param> ParseParams(Parser.Result result)
        {
            //paramList => '\(' ( ref id ( ',' ref id )* )? '\)';
            List<Param> paramList = new List<Param>();
            if (result[1].Children.Any())
            {
                Parser.Result paramResult = result[1][0];
                paramList.Add(new Param(Parse(paramResult[0]), paramResult[1].Value));
                paramList.AddRange(paramResult[2].Children.Select(child => new Param(Parse(child[1]), child[2].Value)));
            }
            return paramList;
        }

        private static Expr ParseRetType(Parser.Result result)
        {
            //retType => 'void' | ref
            return result[0].Value == "void" ? new Ref("NoneType") : Parse(result[0]);
        }

        private static List<Expr> ParseBody(Parser.Result result)
        {
            //body => '{' statement* '}';
            return result[1].Children.Select(child => Parse(child)).ToList();
        }
    }
}
