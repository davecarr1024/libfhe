using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Sherp
{
    [TestClass]
    public class LexerTest
    {
        private class LexerResult
        {
            public string Rule { get; private set; }

            public string Value { get; private set; }

            public LexerResult(string rule, string value)
            {
                Rule = rule;
                Value = value;
            }

            public LexerResult(Lexer.Result result)
            {
                Rule = result.Rule.Name;
                Value = result.Value;
            }

            public override bool Equals(object obj)
            {
                LexerResult result = obj as LexerResult;
                return result != null && result.Rule == Rule && result.Value == Value;
            }

            public override int GetHashCode()
            {
                return base.GetHashCode();
            }
        }

        [TestMethod]
        public void TestLexer()
        {
            Lexer.Lexer lexer = new Lexer.Lexer(
                new Lexer.Rule("int", @"\d+", true),
                new Lexer.Rule("id", @"[a-zA-Z_][a-zA-Z0-9_]+", true),
                new Lexer.Rule("string", @""".*?""", true),
                new Lexer.Rule("ws", @"\s+", false)
            );
            Assert.IsTrue((new List<LexerResult>()
            {
                new LexerResult("id","foo_3"),
                new LexerResult("int","13"),
                new LexerResult("string",@"""hello"""),
            }).SequenceEqual(lexer.Lex(@"
                foo_3
                13
                ""hello""
            ").Select(result => new LexerResult(result))));
        }
    }
}
