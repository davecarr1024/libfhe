using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sharpy.Interpreter
{
    public static class Interpreter
    {
        public static bool CanConvert(Vals.Val fromType, Vals.Val toType)
        {
            return fromType == toType;
        }

        public static Vals.Val Eval(string input)
        {
            Parser.Parser parser = new Parser.Parser(@"
                id = '[a-zA-Z][a-zA-Z0-9_-]*';
                int = '\d+';
                str = '"".*?""';
                ws ~= '\s+';
                program => statement+;
                statement => varCtorDecl | varDecl | exprStatement;
                varCtorDecl => id id '\(' ( expr ( ',' expr )* )? '\)' ';';
                varDecl => id id ( '=' expr )? ';';
                exprStatement => expr ';';
                expr => binaryOperation | unaryOperation | parenExpr | call | ref | int | str;
                call => ref '\(' ( expr ( ',' expr )* )? '\)';
                ref => id ( '\.' id )*;
                unaryOperation => ( '!' | '\-' | '\+\+' | '\-\-' ) operand;
                binaryOperation => operand ( '\+' | '\-' | '\*' | '\/' | '=' | '==' | '!=' | '<' | '<=' | '>' | '>=' | '\+=' | '\-=' | '\*=' | '\/=' ) operand;
                operand => parenExpr | unaryOperation | call | ref | int | str;
                parenExpr => '\(' expr '\)';
            ");

            Scope scope = new Scope(null);
            scope.Add("None", new Vals.NoneType());
            scope.Add("True", new Vals.Bool(true));
            scope.Add("False", new Vals.Bool(false));

            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes().Where(t => t.GetCustomAttributes().OfType<Attrs.BuiltinClass>().Any()))
            {
                scope.Add(type.Name, Vals.BuiltinClass.Bind(type));
            }

            return parser.Apply(input).Children.Select(child => Parse(child).Eval(scope)).Last();
        }

        private static Exprs.Expr Parse(Parser.Result result)
        {
            switch (result.Type)
            {
                case "statement":
                case "exprStatement":
                case "expr":
                    return Parse(result[0]);
                case "ref":
                    //ref => id ( '\.' id )*;
                    return new Exprs.Ref(result[0].Value, result[1].Children.Select(child => child[1].Value).ToArray());
                case "int":
                    return new Exprs.Int(int.Parse(result.Value));
                case "str":
                    return new Exprs.Str(result.Value.Substring(1, result.Value.Length - 2));
                case "call":
                    {
                        //call => ref '\(' ( expr ( ',' expr )* )? '\)';
                        List<Exprs.Expr> args = new List<Exprs.Expr>();
                        if (result[2].Children.Any())
                        {
                            Parser.Result argResult = result[2][0];
                            args.Add(Parse(argResult[0]));
                            args.AddRange(argResult[1].Children.Select(child => Parse(child[1])));
                        }
                        return new Exprs.Call(Parse(result[0]) as Exprs.Ref, args);
                    }
                case "unaryOperation":
                    //unaryOperation => ( '\!' | '\-' ) expr;
                    return new Exprs.UnaryOperation(result[0][0].Value, Parse(result[1]));
                case "varDecl":
                    //varDecl => id id ( '=' expr )? ';';
                    if (result[2].Children.Any())
                    {
                        return new Exprs.Decl(result[0].Value, result[1].Value, Parse(result[2][0][1]), null);
                    }
                    else
                    {
                        return new Exprs.Decl(result[0].Value, result[1].Value, null, null);
                    }
                case "varCtorDecl":
                    //varCtorDecl => id id '\(' ( expr ( ',' expr )* )? '\)' ';';
                    {
                        List<Exprs.Expr> args = new List<Exprs.Expr>();
                        if (result[3].Children.Any())
                        {
                            Parser.Result argResult = result[3][0];
                            args.Add(Parse(argResult[0]));
                            args.AddRange(argResult[1].Children.Select(child => Parse(child[1])));
                        }
                        return new Exprs.Decl(result[0].Value, result[1].Value, null, args);
                    }
                default:
                    throw new NotImplementedException(result.Type);
            }
        }
    }
}
