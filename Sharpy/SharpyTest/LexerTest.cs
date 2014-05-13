using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Sharpy
{
    [TestClass]
    public class LexerTest
    {
        private class LexResult
        {
            public string Rule { get; private set; }

            public string Value { get; private set; }

            public LexResult(Lexer.Result result)
            {
                Rule = result.Rule.Name;
                Value = result.Value;
            }

            public LexResult(string rule, string value)
            {
                Rule = rule;
                Value = value;
            }

            public override bool Equals(object obj)
            {
                LexResult value = obj as LexResult;
                return value != null && value.Rule == Rule && value.Value == Value;
            }

            public override int GetHashCode()
            {
                return base.GetHashCode();
            }

            public override string ToString()
            {
                return string.Format("LexResult({0},{1})", Rule, Value);
            }
        }

        [TestMethod]
        public void Lex()
        {
            Assert.IsTrue(new List<LexResult>()
            {
                new LexResult("lparen","("),
                new LexResult("id","foo"),
                new LexResult("int","1"),
                new LexResult("str","\"bar\""),
                new LexResult("rparen",")"),
            }.SequenceEqual(new Lexer.Lexer(
                new Lexer.Rule("lparen", @"\(", true),
                new Lexer.Rule("rparen", @"\)", true),
                new Lexer.Rule("id", @"[a-zA-Z]*", true),
                new Lexer.Rule("int", @"\d+", true),
                new Lexer.Rule("str", @""".*?""", true),
                new Lexer.Rule("ws", @"\s+", false)
            ).Apply(@"( foo 1 ""bar"")")
            .Select(result => new LexResult(result))));
        }
    }
}
