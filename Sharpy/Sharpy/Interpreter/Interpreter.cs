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
                id = '[a-zA-Z_][a-zA-Z0-9_-]*';
                int = '\d+';
                str = '"".*?""';
                ws ~= '\s+';
                program => statement+;
                statement => ctor | classDecl | funcDecl | returnStatement | varCtorDecl | varDecl | exprStatement;
                varCtorDecl => mods ref id argList ';';
                varDecl => mods ref id ( '=' expr )? ';';
                exprStatement => expr ';';
                expr => binaryOperation | operand;
                call => ref argList;
                ref => id ( '\.' id )*;
                unaryOperation => ( '!' | '\-' | '\+\+' | '\-\-' ) operand;
                binaryOperation => operand ( '\+' | '\-' | '\*' | '\/' | '=' | '==' | '!=' | '<' | '<=' | '>' | '>=' | '\+=' | '\-=' | '\*=' | '\/=' | '\|\|' | '&&' ) operand;
                operand => parenExpr | unaryOperation | call | ref | int | str;
                parenExpr => '\(' expr '\)';
                funcDecl => mods retType id paramList body;
                returnStatement => 'return' expr? ';';
                mods => ( 'static' | 'private' | 'public' )*;
                classDecl => mods 'class' id body;
                ctor => mods id paramList body;
                paramList => '\(' ( ref id ( ',' ref id )* )? '\)';
                argList => '\(' ( expr ( ',' expr )* )? '\)';
                body => '{' statement* '}';
                retType => 'void' | ref;
            ");

            Scope scope = new Scope(null);
            scope.Add("none", new Vals.NoneType());
            scope.Add("true", new Vals.Bool(true));
            scope.Add("false", new Vals.Bool(false));

            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes().Where(t => t.GetCustomAttributes().OfType<Attrs.BuiltinClass>().Any()))
            {
                Vals.BuiltinClass c = Vals.BuiltinClass.Bind(type);
                scope.Add(type.Name, c);
                foreach (Attrs.BuiltinClass attr in type.GetCustomAttributes().OfType<Attrs.BuiltinClass>())
                {
                    foreach (string name in attr.Names)
                    {
                        scope.Add(name, c);
                    }
                }
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

        public static bool CanApply(Vals.Val obj, string func, params Vals.Val[] argTypes)
        {
            return obj.Scope != null && obj.Scope.CanApply(func, argTypes);
        }

        public static Vals.Val Apply(Vals.Val obj, string func, params Vals.Val[] args)
        {
            if (obj.Scope == null)
            {
                throw new Exception("obj " + obj + " of type " + obj.Type + " can't apply func " + func + ": no scope");
            }
            else if (!obj.Scope.CanApply(func, args.Select(arg => arg.Type).ToArray()))
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
                    //call => expr argList;
                    return new Exprs.Call(Parse(result[0]), ParseArgs(result[1]));
                case "unaryOperation":
                    //unaryOperation => ( '\!' | '\-' ) expr;
                    return new Exprs.UnaryOperation(result[0][0].Value, Parse(result[1]));
                case "varDecl":
                    //varDecl => mods id id ( '=' expr )? ';';
                    return new Exprs.Decl(ParseMods(result[0]), Parse(result[1]), result[2].Value, result[3].Children.Any() ? Parse(result[3][0][1]) : null, null);
                case "varCtorDecl":
                    //varCtorDecl => mods ref id argList ';';
                    return new Exprs.Decl(ParseMods(result[0]), Parse(result[1]), result[2].Value, null, ParseArgs(result[3]));
                case "binaryOperation":
                    //binaryOperation => operand ( '\+' | '\-' | '\*' | '\/' | '=' | '==' | '!=' | '<' | '<=' | '>' | '>=' | '\+=' | '\-=' | '\*=' | '\/=' ) operand;
                    return new Exprs.BinaryOperation(Parse(result[0]), result[1][0].Value, Parse(result[2]));
                case "returnStatement":
                    //returnStatement => 'return' expr? ';';
                    return new Exprs.ReturnStatement(result[1].Children.Any() ? Parse(result[1][0]) : null);
                case "funcDecl":
                    //funcDecl => mods retType id paramList body;
                    return new Exprs.Func(ParseMods(result[0]), result[2].Value, ParseParams(result[3]), ParseRetType(result[1]), ParseBody(result[4]));
                case "classDecl":
                    //classDecl => mods 'class' id body;
                    return new Exprs.Class(ParseMods(result[0]), result[2].Value, ParseBody(result[3]));
                case "ctor":
                    //ctor => mods id paramList body;
                    return new Exprs.Ctor(ParseMods(result[0]), result[1].Value, ParseParams(result[2]), ParseBody(result[3]));
                default:
                    throw new NotImplementedException(result.Type);
            }
        }

        private static Exprs.Mods ParseMods(Parser.Result result)
        {
            //mods => ( 'static' | 'private' | 'public' )*;
            Exprs.Mods.Perms perm = Exprs.Mods.Perms.Private;
            bool stat = false;
            foreach (string val in result.Children.Select(child => child[0].Value))
            {
                switch (val)
                {
                    case "static":
                        stat = true;
                        break;
                    case "private":
                        perm = Exprs.Mods.Perms.Private;
                        break;
                    case "public":
                        perm = Exprs.Mods.Perms.Public;
                        break;
                }
            }
            return new Exprs.Mods(perm, stat);
        }

        private static List<Exprs.Expr> ParseArgs(Parser.Result result)
        {
            //argList => '\(' ( expr ( ',' expr )* )? '\)';
            List<Exprs.Expr> args = new List<Exprs.Expr>();
            if (result[1].Children.Any())
            {
                Parser.Result argResult = result[1][0];
                args.Add(Parse(argResult[0]));
                args.AddRange(argResult[1].Children.Select(child => Parse(child[1])));
            }
            return args;
        }

        private static List<Exprs.Param> ParseParams(Parser.Result result)
        {
            //paramList => '\(' ( ref id ( ',' ref id )* )? '\)';
            List<Exprs.Param> paramList = new List<Exprs.Param>();
            if (result[1].Children.Any())
            {
                Parser.Result paramResult = result[1][0];
                paramList.Add(new Exprs.Param(Parse(paramResult[0]), paramResult[1].Value));
                paramList.AddRange(paramResult[2].Children.Select(child => new Exprs.Param(Parse(child[1]), child[2].Value)));
            }
            return paramList;
        }

        private static Exprs.Expr ParseRetType(Parser.Result result)
        {
            //retType => 'void' | ref
            return result[0].Value == "void" ? new Exprs.Ref("NoneType") : Parse(result[0]);
        }

        private static List<Exprs.Expr> ParseBody(Parser.Result result)
        {
            //body => '{' statement* '}';
            return result[1].Children.Select(child => Parse(child)).ToList();
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
