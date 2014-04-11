using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sherp
{
    public static class Sherp
    {
        private static Parser.Parser parser;

        public static Scope DefaultScope()
        {
            parser = new Parser.Parser(@"
                lparen = '\(';
                rparen = '\)';
                lbrace = '{';
                rbrace = '}';
                dot = '\.';
                comma = ',';
                semicolon = ';';
                def = 'def';
                class = 'class';
                int = '\d+';
                str = '"".*?""';
                id = '[a-zA-Z_][a-zA-Z0-9_]*';
                ws ~= '\s+';
                program => statement+;
                statement => funcDecl | classDecl | line;
                funcDecl => def id parenExpr compoundStatement;
                classDecl => class id compoundStatement;
                parenExpr => lparen expr rparen;
                compoundStatement => lbrace program rbrace;
                line => expr semicolon;
                expr => call | ref | int | str;
                ref => id refTail;
                refTail => refIter*;
                refIter => dot id;
                call => ref argList compoundStatement;
                parenParamList => emptyParamList | paramList;
                emptyParamList => lparen rparen;
                paramList => lparen id paramListTail rparen;
                paramListTail => paramListIter*;
                paramListIter => comma id;
            ");
            Scope scope = new Scope();
            scope["None"] = new Vals.NoneType();
            scope["False"] = new Vals.Bool(false);
            scope["True"] = new Vals.Bool(true);
            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes().Where(t => t.GetCustomAttributes().Any(attr => attr is BuiltinClass)))
            {
                scope[type.Name] = Vals.Class.Bind(type);
            }
            return scope;
        }

        public static Val Eval(string input)
        {
            return Eval(input, DefaultScope());
        }

        public static Val Eval(string input, Scope scope)
        {
            return parser.Parse(input).Children.Select(expr => Parse(expr).Eval(scope)).Last();
        }

        private static Expr Parse(Parser.Result expr)
        {
            switch (expr.Rule.Name)
            {
                case "statement":
                case "line":
                case "expr":
                    return Parse(expr.Children[0]);
                case "ref":
                    {
                        List<string> ids = new List<string>() { expr.Children[0].Value };
                        return new Exprs.Ref(ids.ToArray());
                    }
                default:
                    throw new Exception("invalid expr " + expr.Rule.Name);
            }
        }
    }
}
