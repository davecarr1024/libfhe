using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using libterp;

namespace interp
{
    class Program
    {
        static void Main(string[] args)
        {
            Lexer lexer = new Lexer(
                new Lexer.Rule("lparen", @"\("),
                new Lexer.Rule("rparen", @"\)"),
                new Lexer.Rule("id", @"[a-zA-Z]\w*"),
                new Lexer.Rule("ws", @"\s+", true)
                );
            List<Lexer.Result> tokens = lexer.Lex("(a)");
            System.Diagnostics.Debug.Assert(
                tokens.Count == 3 && 
                tokens[0].Type == "lparen" && 
                tokens[0].Value == "(" &&
                tokens[1].Type == "id" && 
                tokens[1].Value == "a" &&
                tokens[2].Type == "rparen" && 
                tokens[2].Value == ")"
                );

            //Parser parser = new Parser(
            //    new Parser.Rule(Parser.Rule.Type.And, "expr",
            //        new Parser.Rule(Parser.Rule.Type.Token, "lparen"),
            //        new Parser.Rule(Parser.Rule.Type.Token, "id"),
            //        new Parser.Rule(Parser.Rule.Type.Token, "rparen")),
            //    lexer);

            //Parser.Result result = parser.Parse("( a )");
            //System.Diagnostics.Debug.Assert(result != null);
        }
    }
}
