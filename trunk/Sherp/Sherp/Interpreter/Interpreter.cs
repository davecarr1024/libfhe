using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sherp.Interpreter
{
    public static class Interpreter
    {
        private static Parser.Parser parser;

        public static Scope DefaultScope()
        {
            parser = new Parser.Parser(@"
                id = '[a-zA-Z_][a-zA-Z0-9_-]*';
                int = '\d+';
                str = '"".*?""';
                ws ~= '\s+';
                program => decl*;
                decl => namespaceDecl | classDecl;
                namespaceDecl => 'namespace' id '{' decl* '}';
                classDecl => 'class' id '{' classBody* '}';
                classBody => func | classDecl;
                func => id id '\(' ( id id ( ',' id id )* )? '\)' '{' statement* '}';
                statement => returnStatement | emptyReturnStatement | callStatement | assignmentStatement | declStatement;
                returnStatement => 'return' expr ';';
                emptyReturnStatement => 'return' ';';
                expr => unaryOperation | binaryOperation | call | ref | int | str;
                ref => id ( '\.' id )*;
                callStatement => call ';';
                call => ref '\(' ( expr ( ',' expr )* )? '\)';
                assignmentStatement => ref '=' expr ';';
                declStatement => ref id '=' expr ';';
                binaryOperation => operand binaryOperator operand;
                binaryOperator => '==' | '!=' | '\+' | '-' | '\*' | '/' | '<' | '<=' | '>' | '>=';
                operand => call | ref | int | str | parenExpr;
                parenExpr => '\(' expr '\)';
                unaryOperation => unaryOperator expr;
                unaryOperator => '-' | '!';
            ");
            Scope scope = new Scope();
            scope["None"] = new Vals.NoneType();
            scope["True"] = new Vals.Bool(true);
            scope["False"] = new Vals.Bool(false);

            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes().Where(t => t.GetCustomAttributes().Any(a => a is Attrs.BuiltinClass)))
            {
                scope[type.Name] = Vals.Class.Bind(type);
            }

            return scope;
        }

        public static Vals.Val Eval(string input)
        {
            return Eval(input, DefaultScope());
        }

        public static Vals.Val Eval(string input, Scope scope)
        {
            foreach (Parser.Result expr in parser.Parse(input).Children)
            {
                Parse(expr).Eval(scope);
            }
            Vals.Val func;
            if (scope.Search("Main", out func) && func is Vals.ApplyVal)
            {
                return (func as Vals.ApplyVal).Apply(new List<Exprs.Expr>(), scope);
            }
            else
            {
                return new Vals.NoneType();
            }
        }

        private static Vals.Val Search(string id, Scope scope)
        {
            Vals.Val val;
            if (scope.TryGetValue(id, out val))
            {
                return val;
            }
            else
            {
                foreach (Scope child in scope.Values.OfType<Vals.ScopeVal>().Select(scopeVal => scopeVal.Scope))
                {
                    val = Search(id, child);
                    if (val != null)
                    {
                        return val;
                    }
                }
                return null;
            }
        }

        private static Exprs.Expr Parse(Parser.Result expr)
        {
            switch (expr.Rule.Name)
            {
                case "decl":
                case "classBody":
                case "statement":
                case "expr":
                case "callStatement":
                case "operand":
                    return Parse(expr.Children[0]);
                case "classDecl":
                    //classDecl => 'class' id '{' classBodyExpr* '}';
                    //classBodyExpr => func | classDecl;
                    return new Exprs.Class(expr.Children[1].Value, expr.Children[3].Children.Select(child => Parse(child)).ToList());
                case "func":
                    //func => id id '\(' ( id id ( ',' id id )* )? '\)' '{' statement* '}';
                    List<Exprs.Param> paramList = new List<Exprs.Param>();
                    Parser.Result paramsExpr = expr.Children[3].Children.FirstOrDefault();
                    if (paramsExpr != null)
                    {
                        paramList.Add(new Exprs.Param(paramsExpr.Children[0].Value, paramsExpr.Children[1].Value));
                        paramList.AddRange(paramsExpr.Children[2].Children.Select(child => new Exprs.Param(child.Children[1].Value, child.Children[2].Value)));
                    }
                    return new Exprs.Func(expr.Children[1].Value, expr.Children[0].Value, paramList, expr.Children[6].Children.Select(child => Parse(child)).ToList());
                case "returnStatement":
                    //returnStatement => 'return' expr ';';
                    return new Exprs.ReturnStatement(Parse(expr.Children[1]));
                case "emptyReturnStatement":
                    return new Exprs.ReturnStatement(new Exprs.Direct(new Vals.NoneType()));
                case "ref":
                    //ref => id ( '.' id )*;
                    Exprs.Ref r = new Exprs.Ref(expr.Children[0].Value);
                    r.Ids.AddRange(expr.Children[1].Children.Select(child => child.Children[1].Value).ToArray());
                    return r;
                case "call":
                    //call => ref '\(' ( expr ( ',' expr )* )? '\)';
                    Exprs.Call call = new Exprs.Call(Parse(expr.Children[0]), new List<Exprs.Expr>());
                    Parser.Result argsExpr = expr.Children[2].Children.FirstOrDefault();
                    if (argsExpr != null)
                    {
                        call.Args.Add(Parse(argsExpr.Children[0]));
                        call.Args.AddRange(argsExpr.Children[1].Children.Select(child => Parse(child.Children[1])));
                    }
                    return call;
                case "assignmentStatement":
                    //assignmentStatement => ref '=' expr ';';
                    return new Exprs.BinaryOperation(Exprs.BinaryOperation.Operators.Assign, Parse(expr.Children[0]), Parse(expr.Children[2]));
                case "declStatement":
                    //declStatement => ref id '=' expr ';';
                    return new Exprs.Declaration(Parse(expr.Children[0]), expr.Children[1].Value, Parse(expr.Children[3]));
                case "binaryOperation":
                    //binaryOperation => operand binaryOperator operand;
                    return new Exprs.BinaryOperation(Exprs.BinaryOperation.ParseOperator(expr.Children[1].Children[0].Value), Parse(expr.Children[0]), Parse(expr.Children[2]));
                case "int":
                    return new Exprs.Int(int.Parse(expr.Value));
                case "parenExpr":
                    return Parse(expr.Children[1]);
                case "unaryOperation":
                    //unaryOperation => unaryOperator expr;
                    return new Exprs.UnaryOperation(Exprs.UnaryOperation.ParseOperator(expr.Children[0].Children[0].Value), Parse(expr.Children[1]));
                default:
                    throw new Exception("invalid expr " + expr.Rule.Name);
            }
        }
    }
}
