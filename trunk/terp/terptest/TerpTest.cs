using System;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.IO;
using terp;

namespace terpTest
{
  /// <summary>
  /// Summary description for terpTest
  /// </summary>
  [TestClass]
  public class terpTest
  {
    public terpTest()
    {
      lexer = new Lexer(
        new Lexer.Rule("lp", @"\(", false),
        new Lexer.Rule("rp", @"\)", false),
        new Lexer.Rule("id", @"[a-zA-Z]\w*", false),
        new Lexer.Rule("string", "\"((\\.)|[^\\\\\"])*\"", false),
        new Lexer.Rule("int", @"-?\d+", false),
        new Lexer.Rule("float", @"-?\d+\.\d*", false),
        new Lexer.Rule("ws", @"\s+", true));

      parser = new Parser(lexer,
        new Parser.RuleDef(Parser.RuleType.Or, "expr", "id", "parenExpr", "int", "float", "string"),
        new Parser.RuleDef(Parser.RuleType.And, "parenExpr", "lp", "exprList", "rp"),
        new Parser.RuleDef(Parser.RuleType.ZeroOrMore, "exprList", "expr")
      );
    }

    Lexer lexer;

    Parser parser;

    private TestContext testContextInstance;

    /// <summary>
    ///Gets or sets the test context which provides
    ///information about and functionality for the current test run.
    ///</summary>
    public TestContext TestContext
    {
      get
      {
        return testContextInstance;
      }
      set
      {
        testContextInstance = value;
      }
    }

    #region Additional test attributes
    //
    // You can use the following additional attributes as you write your tests:
    //
    // Use ClassInitialize to run code before running the first test in the class
    // [ClassInitialize()]
    // public static void MyClassInitialize(TestContext testContext) { }
    //
    // Use ClassCleanup to run code after all tests in a class have run
    // [ClassCleanup()]
    // public static void MyClassCleanup() { }
    //
    // Use TestInitialize to run code before running each test 
    // [TestInitialize()]
    // public void MyTestInitialize() { }
    //
    // Use TestCleanup to run code after each test has run
    // [TestCleanup()]
    // public void MyTestCleanup() { }
    //
    #endregion

    [TestMethod]
    public void lex()
    {
      List<Lexer.Result> results = lexer.Lex(" ( a ) ");
      Assert.AreEqual(3, results.Count);
      Assert.AreEqual("lp", results[0].Type);
      Assert.AreEqual("(", results[0].Value);
      Assert.AreEqual("id", results[1].Type);
      Assert.AreEqual("a", results[1].Value);
      Assert.AreEqual("rp", results[2].Type);
      Assert.AreEqual(")", results[2].Value);
    }

    private void CheckResult(Parser.Result result, string type, string value, int numChildren)
    {
      Assert.AreEqual(type, result.Type);
      Assert.AreEqual(value, result.Value);
      Assert.AreEqual(numChildren, result.Children.Count);
    }

    [TestMethod]
    public void parse()
    {
      Assert.AreEqual(5, parser.RootRule.Children.Count);

      Parser.Result result = parser.Parse(" ( a1b2c3 12 3.14 \"hello world\" ) ");

      Parser.Result expected =
        new Parser.Result("expr", null,
          new Parser.Result("parenExpr", null,
            new Parser.Result("lp", "("),
            new Parser.Result("exprList", null,
              new Parser.Result("expr", null,
                new Parser.Result("id", "a1b2c3")
              ),
              new Parser.Result("expr", null,
                new Parser.Result("int", "12")
              ),
              new Parser.Result("expr", null,
                new Parser.Result("float","3.14")
              ),
              new Parser.Result("expr", null,
                new Parser.Result("string", "\"hello world\"")
              )
            ),
            new Parser.Result("rp", ")")
          )
        );

      Assert.IsTrue(result.Equals(expected));
    }

    [TestMethod]
    public void grammar()
    {
      Parser parser = new Parser(File.ReadAllText("../../../terpTest/lisp.trp"));

      Parser.Result result = parser.Parse(" ( a1b2c3 12 3.14 \"hello world\" ) ");

      Parser.Result expected =
        new Parser.Result("expr", null,
          new Parser.Result("parenExpr", null,
            new Parser.Result("lp", "("),
            new Parser.Result("exprList", null,
              new Parser.Result("expr", null,
                new Parser.Result("id", "a1b2c3")
              ),
              new Parser.Result("expr", null,
                new Parser.Result("int", "12")
              ),
              new Parser.Result("expr", null,
                new Parser.Result("float", "3.14")
              ),
              new Parser.Result("expr", null,
                new Parser.Result("string", "\"hello world\"")
              )
            ),
            new Parser.Result("rp", ")")
          )
        );

      Assert.IsTrue(result.Equals(expected));
    }

    private void AssertNear(float f1, float f2)
    {
      Assert.IsTrue(Math.Abs(f1 - f2) <= 1e-5);
    }

    [TestMethod]
    public void lisp()
    {
      LispInterpreter terp = new LispInterpreter("../../../terpTest/lisp.trp");

      LispInterpreter.Value value = terp.Interpret("-12");
      Assert.AreEqual(LispInterpreter.Value.Type.Int, value.type);
      Assert.AreEqual(-12, value.Int);

      value = terp.Interpret("-3.14");
      Assert.AreEqual(LispInterpreter.Value.Type.Float, value.type);
      AssertNear(-3.14f, value.Float);

      value = terp.Interpret("\"herro\"");
      Assert.AreEqual(LispInterpreter.Value.Type.String, value.type);
      Assert.AreEqual("herro", value.String);

      value = terp.Interpret("(add 1 2)");
      Assert.AreEqual(LispInterpreter.Value.Type.Float, value.type);
      AssertNear(3, value.Float);

      value = terp.Interpret("(define foo (a b) (add a (add 3 b))) (foo 2 10)");
      Assert.AreEqual(LispInterpreter.Value.Type.Float, value.type);
      AssertNear(15, value.Float);
    }
  }
}
