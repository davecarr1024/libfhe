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
        public static Vals.Val Eval(string input)
        {
            Parser.Parser parser = new Parser.Parser(@"
                id = '[a-zA-Z][a-zA-Z0-9_-]*';
                int = '\d+';
                str = '"".*?""';
                ws ~= '\s+';
                program => statement+;
                statement => funcDecl | returnStatement | varCtorDecl | varDecl | exprStatement;
                varCtorDecl => id id '\(' ( expr ( ',' expr )* )? '\)' ';';
                varDecl => id id ( '=' expr )? ';';
                exprStatement => expr ';';
                expr => binaryOperation | unaryOperation | parenExpr | call | ref | int | str;
                call => ref '\(' ( expr ( ',' expr )* )? '\)';
                ref => id ( '\.' id )*;
                unaryOperation => ( '!' | '\-' | '\+\+' | '\-\-' ) operand;
                binaryOperation => operand ( '\+' | '\-' | '\*' | '\/' | '=' | '==' | '!=' | '<' | '<=' | '>' | '>=' | '\+=' | '\-=' | '\*=' | '\/=' | '\|\|' | '&&' ) operand;
                operand => parenExpr | unaryOperation | call | ref | int | str;
                parenExpr => '\(' expr '\)';
                funcDecl => ref id '\(' ( ref id ( ',' ref id )* )? '\)' '{' statement* '}';
                returnStatement => 'return' expr? ';';
            ");

            Scope scope = new Scope(null);
            scope.Add("None", new Vals.NoneType());
            scope.Add("True", new Vals.Bool(true));
            scope.Add("False", new Vals.Bool(false));

            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes().Where(t => t.GetCustomAttributes().OfType<Attrs.BuiltinClass>().Any()))
            {
                scope.Add(type.Name, Vals.BuiltinClass.Bind(type));
            }

            return Eval(scope, parser.Apply(input).Children.Select(child => Parse(child)).ToArray());
        }

        public static Vals.Val Eval(Scope scope, params Exprs.Expr[] exprs)
        {
            foreach (Exprs.Expr expr in exprs)
            {
                Vals.Val val = expr.Eval(scope);
                if (val.IsReturn)
                {
                    return val;
                }
            }
            return new Vals.NoneType();
        }

        public static bool CanApply(Vals.Val obj, string func, params Vals.Val[] args)
        {
            return obj.Scope != null && obj.Scope.CanApply(func, args);
        }

        public static Vals.Val Apply(Vals.Val obj, string func, params Vals.Val[] args)
        {
            if (obj.Scope == null)
            {
                throw new Exception("obj " + obj + " of type " + obj.Type + " can't apply func " + func + ": no scope");
            }
            else if (!obj.Scope.CanApply(func, args))
            {
                throw new Exception("obj " + obj + " of type " + obj.Type + " can't apply func " + func + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
            else
            {
                return obj.Scope.Apply(func, args);
            }
        }

        private static Exprs.Expr Parse(Parser.Result result)
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
                case "binaryOperation":
                    //binaryOperation => operand ( '\+' | '\-' | '\*' | '\/' | '=' | '==' | '!=' | '<' | '<=' | '>' | '>=' | '\+=' | '\-=' | '\*=' | '\/=' ) operand;
                    return new Exprs.BinaryOperation(Parse(result[0]), result[1][0].Value, Parse(result[2]));
                case "returnStatement":
                    //returnStatement => 'return' expr? ';';
                    if (result[1].Children.Any())
                    {
                        return new Exprs.ReturnStatement(Parse(result[1][0]));
                    }
                    else
                    {
                        return new Exprs.ReturnStatement(null);
                    }
                case "funcDecl":
                    //funcDecl => ref id '\(' ( ref id ( ',' ref id )* )? '\)' '{' statement* '}';
                    {
                        List<Exprs.Param> paramList = new List<Exprs.Param>();
                        if (result[3].Children.Any())
                        {
                            Parser.Result paramResult = result[3][0];
                            paramList.Add(new Exprs.Param(Parse(paramResult[0]), paramResult[1].Value));
                            paramList.AddRange(paramResult[2].Children.Select(child => new Exprs.Param(Parse(child[1]), child[2].Value)));
                        }
                        return new Exprs.Func(result[1].Value, paramList, Parse(result[0]), result[6].Children.Select(child => Parse(child)).ToList());
                    }
                default:
                    throw new NotImplementedException(result.Type);
            }
        }

        public static bool CanConvert(Vals.Val fromType, Vals.Val toType)
        {
            return fromType == toType || toType.CanApply(fromType);
        }

        public static Vals.Val Convert(Vals.Val val, Vals.Val toType)
        {
            if (val.Type == toType)
            {
                return val;
            }
            else if (toType.CanApply(val.Type))
            {
                return toType.Apply(val);
            }
            else
            {
                throw new Exception("unable to convert val " + val + " to type " + toType);
            }
        }
    }
}
