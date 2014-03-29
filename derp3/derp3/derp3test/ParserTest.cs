﻿using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using derp3;

namespace derp3test
{
    [TestClass]
    public class ParserTest
    {
        private class ParseResult
        {
            public string RuleName { get; set; }

            public string Value { get; set; }

            public List<ParseResult> Children { get; set; }

            public ParseResult(string ruleName, string value)
            {
                RuleName = ruleName;
                Value = value;
                Children = null;
            }

            public ParseResult(string ruleName, params ParseResult[] children)
            {
                RuleName = ruleName;
                Value = null;
                Children = children.ToList();
            }

            public ParseResult(Parser.Result result)
            {
                RuleName = result.Rule.Name;
                Value = result.Value;
                Children = result.Children.Select(childResult => new ParseResult(childResult)).ToList();
            }

            public override bool Equals(object obj)
            {
                ParseResult result = obj as ParseResult;
                return result != null &&
                    result.RuleName == RuleName &&
                    result.Value == Value &&
                    ((result.Children == null && Children == null) ||
                    (result.Children.Count == Children.Count && result.Children.SequenceEqual(Children)));
            }
        }

        [TestMethod]
        public void TestParse()
        {
            Lexer lexer = new Lexer(
                new Lexer.Rule("lparen", @"\(", true),
                new Lexer.Rule("rparen", @"\)", true),
                new Lexer.Rule("num", @"\d+", true),
                new Lexer.Rule("id", @"\w+", true),
                new Lexer.Rule("ws", @"\s+", false));
            Parser parser = new Parser(lexer,
                new Parser.Rule(Parser.Rule.Types.AND, "expr",
                    new Parser.Rule(Parser.Rule.Types.TERMINAL, "lparen"),
                    new Parser.Rule(Parser.Rule.Types.ONEORMORE, "exprContent",
                        new Parser.Rule(Parser.Rule.Types.OR, "val",
                            new Parser.Rule(Parser.Rule.Types.TERMINAL, "id"),
                            new Parser.Rule(Parser.Rule.Types.TERMINAL, "num")
                            )
                        ),
                    new Parser.Rule(Parser.Rule.Types.TERMINAL, "rparen")
                    )
                );
            Assert.AreEqual(
                new ParseResult("expr",
                    new ParseResult("lparen", "("),
                    new ParseResult("exprContent",
                        new ParseResult("val",
                            new ParseResult("id", "foo")
                            ),
                        new ParseResult("val",
                            new ParseResult("num", "3")
                            )
                        ),
                    new ParseResult("rparen", ")")
                    ),
                parser.Parse(@"( foo 3 )")
                );
        }
    }
}
