using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Sharpy
{
    [TestClass]
    public class ParserTest
    {
        private class ParseResult
        {
            public string Rule { get; private set; }

            public string Value { get; private set; }

            public List<ParseResult> Children { get; private set; }

            public ParseResult(string rule, string value)
            {
                Rule = rule;
                Value = value;
                Children = new List<ParseResult>();
            }

            public ParseResult(string rule, params ParseResult[] children)
            {
                Rule = rule;
                Value = null;
                Children = children.ToList();
            }

            public ParseResult(Parser.Result result)
            {
                Rule = result.Rule.Name;
                Value = result.Value;
                Children = result.Children.Select(child => new ParseResult(child)).ToList();
            }

            public override bool Equals(object obj)
            {
                ParseResult value = obj as ParseResult;
                return value != null && value.Rule == Rule && value.Value == Value && value.Children.SequenceEqual(Children);
            }

            public override int GetHashCode()
            {
                return base.GetHashCode();
            }
        }

        [TestMethod]
        public void DirectParse()
        {
            Assert.AreEqual(
                new ParseResult("program",
                    new ParseResult("compoundExpr",
                        new ParseResult("lparen", "("),
                        new ParseResult("compoundExprContent",
                            new ParseResult("expr",
                                new ParseResult("id", "foo")
                            ),
                            new ParseResult("expr",
                                new ParseResult("int", "12")
                            ),
                            new ParseResult("expr",
                                new ParseResult("str", "\"hello\"")
                            )
                        ),
                        new ParseResult("rparen", ")")
                    )
                ),
                new ParseResult(new Parser.Parser(
                    new Lexer.Lexer(
                        new Lexer.Rule("lparen", @"\(", true),
                        new Lexer.Rule("rparen", @"\)", true),
                        new Lexer.Rule("id", @"[a-zA-Z]*", true),
                        new Lexer.Rule("int", @"\d+", true),
                        new Lexer.Rule("str", @""".*?""", true),
                        new Lexer.Rule("ws", @"\s+", false)
                    ),
                    new Parser.Rule(Parser.Rule.Types.OneOrMore, "program",
                        new Parser.Rule(Parser.Rule.Types.And, "compoundExpr",
                            new Parser.Rule(Parser.Rule.Types.Terminal, "lparen"),
                            new Parser.Rule(Parser.Rule.Types.ZeroOrMore, "compoundExprContent",
                                new Parser.Rule(Parser.Rule.Types.Or, "expr",
                                    new Parser.Rule(Parser.Rule.Types.Terminal, "id"),
                                    new Parser.Rule(Parser.Rule.Types.Terminal, "int"),
                                    new Parser.Rule(Parser.Rule.Types.Terminal, "str")
                                )
                            ),
                            new Parser.Rule(Parser.Rule.Types.Terminal, "rparen")
                        )
                    )
                ).Apply(@"( foo 12 ""hello"")")));

        }

        [TestMethod]
        public void ExprParse()
        {
            Assert.AreEqual(
                new ParseResult("program",
                    new ParseResult("expr",
                        new ParseResult("compoundExpr",
                            new ParseResult(@"\(", "("),
                            new ParseResult(null,
                                new ParseResult("expr",
                                    new ParseResult("id", "foo")
                                ),
                                new ParseResult("expr",
                                    new ParseResult("int", "12")
                                ),
                                new ParseResult("expr",
                                    new ParseResult("str", "\"hello\"")
                                )
                            ),
                            new ParseResult(@"\)", ")")
                        )
                    )
                ),
                new ParseResult(
                    new Parser.Parser(
                        new Lexer.Lexer(
                            new Lexer.Rule("id", @"[a-zA-Z]*", true),
                            new Lexer.Rule("int", @"\d+", true),
                            new Lexer.Rule("str", @""".*?""", true),
                            new Lexer.Rule("ws", @"\s+", false)
                        ),
                        new Parser.Exprs.Def(Parser.Rule.Types.OneOrMore, "program",
                            new Parser.Exprs.Ref("expr")
                        ),
                        new Parser.Exprs.Def(Parser.Rule.Types.Or, "expr",
                            new Parser.Exprs.Ref("compoundExpr"),
                            new Parser.Exprs.Ref("id"),
                            new Parser.Exprs.Ref("int"),
                            new Parser.Exprs.Ref("str")
                        ),
                        new Parser.Exprs.Def(Parser.Rule.Types.And, "compoundExpr",
                            new Parser.Exprs.Token(@"\("),
                            new Parser.Exprs.Def(Parser.Rule.Types.OneOrMore, null,
                                new Parser.Exprs.Ref("expr")
                            ),
                            new Parser.Exprs.Token(@"\)")
                        )
                    ).Apply(@"( foo 12 ""hello"")")
                )
            );
        }

        [TestMethod]
        public void BootstrapParse()
        {
            Assert.AreEqual(
                new ParseResult("program",
                    new ParseResult("expr",
                        new ParseResult("compoundExpr",
                            new ParseResult(@"\(", "("),
                            new ParseResult(null,
                                new ParseResult("expr",
                                    new ParseResult("id", "foo")
                                ),
                                new ParseResult("expr",
                                    new ParseResult("int", "12")
                                ),
                                new ParseResult("expr",
                                    new ParseResult("str", "\"hello\"")
                                )
                            ),
                            new ParseResult(@"\)", ")")
                        )
                    )
                ),
                new ParseResult(
                    new Parser.Parser( @"
                        id = '[a-zA-Z]+';
                        int = '\d+';
                        str = '"".*?""';
                        ws ~= '\s+';
                        program => expr+;
                        expr => compoundExpr | id | int | str;
                        compoundExpr => '\(' expr+ '\)';
                    ").Apply(@"( foo 12 ""hello"")")
                )
            );
        }
    }
}
