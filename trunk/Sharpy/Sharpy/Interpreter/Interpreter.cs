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
                statement => decl | returnStatement | exprStatement;
                returnStatement => 'return' expr ';';
                exprStatement => expr ';';
                expr => binaryOperation | unaryOperation | call | ref | int | str;
                ref => id ( '\.' id )*;
                call => ref argList;
                argList => '\(' ( expr ( ',' expr )* )? '\)';
                decl => defDecl | valDecl | argsDecl;
                defDecl => ref id ';';
                valDecl => ref id '=' expr ';';
                argsDecl => ref id argList ';';
                binaryOperation => operand ( '==' | '=' ) operand;
                operand => unaryOperation | call | ref | int | str;
                unaryOperation => ( '!' | '-' ) operand;
            ");

            Scope scope = new Scope();
            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes().Where(t => t.GetCustomAttributes().OfType<Attrs.BuiltinClass>().Any()))
            {
                string name = type.GetCustomAttributes().OfType<Attrs.BuiltinClass>().First().Name;
                if (string.IsNullOrEmpty(name))
                {
                    name = type.Name;
                }
                scope.Add(name, Vals.BuiltinClass.Bind(type));
            }
            scope.Add("None", new Vals.NoneType());
            scope.Add("true", new Vals.Bool(true));
            scope.Add("false", new Vals.Bool(false));

            return Eval(scope, parser.Apply(input).Children.Select(child => Exprs.Expr.Parse(child)).ToArray());
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
    }
}
