using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using derp3;

namespace derp3test
{
    [TestClass]
    public class LexerTest
    {
        private class LexResult
        {
            public string RuleName { get; set; }

            public string Value { get; set; }

            public LexResult(Lexer.Result result)
            {
                RuleName = result.Rule.Name;
                Value = result.Value;
            }

            public LexResult(string ruleName, string value)
            {
                RuleName = ruleName;
                Value = value;
            }

            public override bool Equals(object obj)
            {
                return obj is LexResult && (obj as LexResult).RuleName == RuleName && (obj as LexResult).Value == Value;
            }

            public override int GetHashCode()
            {
                return base.GetHashCode();
            }
        }

        [TestMethod]
        public void TestLex()
        {
            Lexer lexer = new Lexer(new Lexer.Rule("num", @"\d+", true),
                    new Lexer.Rule("id", @"\w+", true),
                    new Lexer.Rule("ws", @"\s+", false));
            CollectionAssert.AreEqual(new List<LexResult>()
            {
                new LexResult("num","1"),
                new LexResult("id","foo"),
            }, lexer.Lex(@"1 foo").Select(result => new LexResult(result)).ToList());
        }
    }
}
