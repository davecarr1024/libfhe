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
                scope.Add(c.Name, c);
            }

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
