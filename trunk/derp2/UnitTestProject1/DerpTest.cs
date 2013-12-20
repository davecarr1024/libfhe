using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using derp;

namespace UnitTestProject1
{
    [TestClass]
    public class DerpTest
    {
        [TestMethod]
        public void Lex()
        {
            Lexer lexer = new Lexer()
            {
                Rules = new List<Lexer.Rule>()
                {
                    new Lexer.Rule() { Name = "lparen", Pattern = @"\(", Include = true },
                    new Lexer.Rule() { Name = "rparen", Pattern = @"\)", Include = true },
                    new Lexer.Rule() { Name = "num", Pattern = @"\d+", Include = true },
                    new Lexer.Rule() { Name = "id", Pattern = @"\w+", Include = true },
                    new Lexer.Rule() { Name = "ws", Pattern = @"\s+", Include = false },
                }
            };
            CollectionAssert.AreEqual(
                new List<Tuple<string, string>>() 
                { 
                    new Tuple<string,string>( "lparen", "(" ), 
                    new Tuple<string,string>( "id", "a" ), 
                    new Tuple<string,string>( "num", "1" ), 
                    new Tuple<string,string>( "rparen", ")" ), 
                },
                lexer.Lex("( a 1 )").Select(result => new Tuple<string, string>(result.Rule.Name, result.Value)).ToList()
            );
        }

        [TestMethod]
        public void DirectParse()
        {
            Parser parser = new Parser()
            {
                Lexer = new Lexer()
                {
                    Rules = new List<Lexer.Rule>()
                    {
                        new Lexer.Rule() { Name = "lparen", Pattern = @"\(", Include = true },
                        new Lexer.Rule() { Name = "rparen", Pattern = @"\)", Include = true },
                        new Lexer.Rule() { Name = "num", Pattern = @"\d+", Include = true },
                        new Lexer.Rule() { Name = "id", Pattern = @"\w+", Include = true },
                        new Lexer.Rule() { Name = "ws", Pattern = @"\s+", Include = false },
                    }
                },
                Root = new Parser.Rule()
                {
                    Name = "program",
                    Type = Parser.Rule.Types.OneOrMore,
                    Children = new List<Parser.Rule>()
                    {
                        new Parser.Rule()
                        {
                        },
                    },
                },
            };
        }
    }
}
