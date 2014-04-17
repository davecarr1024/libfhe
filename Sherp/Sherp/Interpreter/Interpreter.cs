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
        //        private static Parser.Parser parser = new Parser.Parser(@"
        //            id = '[a-zA-Z_][a-zA-Z0-9_-]*';
        //            int = '\d+';
        //            str = '"".*?""';
        //            ws ~= '\s+';
        //            program => ( namespace | class )*;
        //            namespace => 'namespace' id '{' ( namespace | class )* '}';
        //            class => 'class' id '{' ( namespace | class | func )* '}';
        //            func => id id '(' params ')' '{' statement* '}';
        //        ");

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
                statement => returnStatement | ( expr ';' );
                returnStatement => 'return' expr ';';
                expr => id | int | str;
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
                default:
                    throw new Exception("invalid expr " + expr.Rule.Name);
            }
        }
    }
}
