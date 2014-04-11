using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Sherp
{
    [TestClass]
    public class ParserTest
    {
        private class ParserResult
        {
            public string Rule { get; private set; }

            public string Value { get; private set; }

            public List<ParserResult> Children { get; private set; }

            public ParserResult(string rule, string value)
            {
                Rule = rule;
                Value = value;
                Children = new List<ParserResult>();
            }

            public ParserResult(string rule, params ParserResult[] children)
            {
                Rule = rule;
                Value = null;
                Children = children.ToList();
            }

            public ParserResult(Parser.Result result)
            {
                Rule = result.Rule.Name;
                Value = result.Value;
                Children = result.Children.Select(child => new ParserResult(child)).ToList();
            }

            public override bool Equals(object obj)
            {
                ParserResult result = obj as ParserResult;
                return result != null && result.Rule == Rule && result.Value == Value && result.Children.SequenceEqual(Children);
            }

            public override int GetHashCode()
            {
                return base.GetHashCode();
            }
        }

        [TestMethod]
        public void TestDirectParser()
        {
            Assert.AreEqual(
                new ParserResult("program",
                    new ParserResult("expr",
                        new ParserResult("lparen", "("),
                        new ParserResult("exprContent"),
                        new ParserResult("rparen", ")")
                    ),
                    new ParserResult("expr",
                        new ParserResult("lparen", "("),
                        new ParserResult("exprContent",
                            new ParserResult("exprContentIter",
                                new ParserResult("id", "foo")
                            ),
                            new ParserResult("exprContentIter",
                                new ParserResult("str", "\"bar\"")
                            ),
                            new ParserResult("exprContentIter",
                                new ParserResult("int", "9")
                            )
                        ),
                        new ParserResult("rparen", ")")
                    )
                ),
                new ParserResult(
                    (new Parser.Parser(
                        new Lexer.Lexer(
                            new Lexer.Rule("lparen", @"\(", true),
                            new Lexer.Rule("rparen", @"\)", true),
                            new Lexer.Rule("id", @"[a-zA-Z_][a-zA-Z0-9_]*", true),
                            new Lexer.Rule("int", @"\d+", true),
                            new Lexer.Rule("str", @""".*?""", true),
                            new Lexer.Rule("ws", @"\s+", false)
                        ),
                        new Parser.Rule("program", Parser.Rule.Types.OneOrMore,
                            new Parser.Rule("expr", Parser.Rule.Types.And,
                                new Parser.Rule("lparen", Parser.Rule.Types.Terminal),
                                new Parser.Rule("exprContent", Parser.Rule.Types.ZeroOrMore,
                                    new Parser.Rule("exprContentIter", Parser.Rule.Types.Or,
                                        new Parser.Rule("id", Parser.Rule.Types.Terminal),
                                        new Parser.Rule("int", Parser.Rule.Types.Terminal),
                                        new Parser.Rule("str", Parser.Rule.Types.Terminal)
                                    )
                                ),
                                new Parser.Rule("rparen", Parser.Rule.Types.Terminal)
                            )
                        )
                    )).Parse(@"() (foo ""bar"" 9)")
                )
            );
        }

        [TestMethod]
        public void TestStrParser()
        {
            Assert.AreEqual(
                new ParserResult("program",
                    new ParserResult("expr",
                        new ParserResult("parenExpr",
                            new ParserResult("lparen", "("),
                            new ParserResult("program",
                                new ParserResult("expr",
                                    new ParserResult("id", "foo")
                                ),
                                new ParserResult("expr",
                                    new ParserResult("str", "\"bar\"")
                                ),
                                new ParserResult("expr",
                                    new ParserResult("int", "9")
                                )
                            ),
                            new ParserResult("rparen", ")")
                        )
                    )
                ),
                new ParserResult(
                    (new Parser.Parser(@"
                        lparen = '\(';
                        rparen = '\)';
                        id = '[a-zA-Z_][a-zA-Z0-9_]*';
                        int = '\d+';
                        str = '"".*?""';
                        ws ~= '\s+';
                        program => expr+;
                        expr => parenExpr | int | str | id;
                        parenExpr => lparen program rparen;
                    ")).Parse(@"(foo ""bar"" 9)")
                )
            );
        }
    }
}
