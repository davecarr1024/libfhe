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
            Lexer lexer = new Lexer(
                new Lexer.Rule( "lparen", @"\(", true ),
                new Lexer.Rule( "rparen", @"\)", true ),
                new Lexer.Rule( "num", @"\d+", true ),
                new Lexer.Rule( "id", @"\w+", true ),
                new Lexer.Rule( "ws", @"\s+", false )
            );
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
            Parser parser = new Parser(
                new Lexer(
                    new Lexer.Rule("lparen", @"\(", true),
                    new Lexer.Rule("rparen", @"\)", true),
                    new Lexer.Rule("num", @"\d+", true),
                    new Lexer.Rule("id", @"\w+", true),
                    new Lexer.Rule("ws", @"\s+", false)
                ),
                new Parser.Rule("program", Parser.Rule.Types.OneOrMore,
                    new Parser.Rule("expr", Parser.Rule.Types.And,
                        new Parser.Rule("lparen", Parser.Rule.Types.Terminal),
                        new Parser.Rule("valList", Parser.Rule.Types.OneOrMore,
                            new Parser.Rule("val", Parser.Rule.Types.Or,
                                new Parser.Rule("num", Parser.Rule.Types.Terminal),
                                new Parser.Rule("id", Parser.Rule.Types.Terminal)
                            )
                        ),
                        new Parser.Rule("rparen", Parser.Rule.Types.Terminal)
                    )
                )
            );
            Assert.AreEqual(new ParseResult("program", null,
                                new ParseResult("expr", null,
                                    new ParseResult("lparen", "("),
                                    new ParseResult("valList", null,
                                        new ParseResult("val", null,
                                            new ParseResult("id", "a")
                                        ),
                                        new ParseResult("val", null,
                                            new ParseResult("num", "1")
                                        )
                                    ),
                                    new ParseResult("rparen", ")")
                                )
                            ),
                            new ParseResult(parser.Parse("( a 1 )")));
        }

        private class ParseResult
        {
            public string Type { get; set; }

            public string Value { get; set; }

            public List<ParseResult> Children { get; set; }

            public ParseResult(Parser.Result result)
            {
                Type = result.Rule.Name;
                Value = result.Value;
                Children = result.Children.Select(childResult => new ParseResult(childResult)).ToList();
            }

            public ParseResult(string type, string value, params ParseResult[] children)
            {
                Type = type;
                Value = value;
                Children = children.ToList();
            }

            public override bool Equals(object obj)
            {
                return obj is ParseResult &&
                    (obj as ParseResult).Type == Type &&
                    (obj as ParseResult).Value == Value &&
                    Enumerable.SequenceEqual((obj as ParseResult).Children, Children);
            }

            public override int GetHashCode()
            {
                return base.GetHashCode();
            }

            private string ToString(int numTabs)
            {
                return string.Concat(Enumerable.Repeat("\t", numTabs)) +
                    "ParseResult( " + Type + ", " + (Value ?? "null") + " )\n" +
                    string.Concat(Children.Select(child => child.ToString(numTabs + 1)));
            }

            public override string ToString()
            {
                return ToString(0);
            }
        }

        [TestMethod]
        public void IndirectParse()
        {
            Parser parser = Parser.Load(@"
lparen = ""\("";
rparen = ""\)"";
num = ""\d+"";
id = ""\w+"";
ws *= ""\s+"";

exprList => expr+;
expr => listExpr | id | num;
listExpr => lparen exprList rparen;
            ");
            Assert.AreEqual(
                new ParseResult("exprList", null,
                    new ParseResult("expr", null,
                        new ParseResult("listExpr", null,
                            new ParseResult("lparen", "("),
                            new ParseResult("exprList", null,
                                new ParseResult("expr", null,
                                    new ParseResult("id", "a")
                                ),
                                new ParseResult("expr", null,
                                    new ParseResult("num", "1")
                                )
                            ),
                            new ParseResult("rparen", ")")
                        )
                    )
                ),
                new ParseResult(parser.Parse("( a 1 )")));
        }
    }
}
