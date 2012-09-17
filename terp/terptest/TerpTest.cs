using System;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Text.RegularExpressions;
using System.Collections.ObjectModel;
using libterp;

namespace terptest
{
  /// <summary>
  /// Summary description for TerpTest
  /// </summary>
  [TestClass]
  public class TerpTest
  {
    public TerpTest()
    {
      //
      // TODO: Add constructor logic here
      //
    }

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

    //[TestMethod]
    public void LexTest()
    {
      Lexer lexer = new Lexer(
        new Lexer.Rule("whitespace", @"\s+"),
        new Lexer.Rule("nonwhitespace", @"\w+")
        );

      List<Lexer.Token> tokens = lexer.Lex(" abc\t");
      Assert.AreEqual(3, tokens.Count);
      Assert.AreEqual("whitespace", tokens[0].Type);
      Assert.AreEqual(" ", tokens[0].Value);
      Assert.AreEqual("nonwhitespace", tokens[1].Type);
      Assert.AreEqual("abc", tokens[1].Value);
      Assert.AreEqual("whitespace", tokens[2].Type);
      Assert.AreEqual("\t", tokens[2].Value);
    }

    //[TestMethod]
    public void ParseTest()
    {
      Lexer lexer = new Lexer(
        new Lexer.Rule("whitespace", @"\s+", true),
        new Lexer.Rule("leftParen", @"\("),
        new Lexer.Rule("rightParen", @"\)"),
        new Lexer.Rule("id", @"[\D\w]\w*")
        );


      Parser parser = new Parser(
        new Parser.Rule(Parser.Rule.Type.And,"expression",
          new Parser.Rule(Parser.Rule.Type.Token,"leftParen"),
          new Parser.Rule(Parser.Rule.Type.OneOrMore,"expressionBody",
            new Parser.Rule(Parser.Rule.Type.Token,"id")
            ),
          new Parser.Rule(Parser.Rule.Type.Token,"rightParen")
        ),
        lexer
      );

      Parser.Result result = parser.Parse("( a )");
      Assert.IsNotNull(result);
      Assert.AreEqual("expression", result.Type);
      Assert.AreEqual(3, result.Children.Count);
      Assert.AreEqual("leftParen", result.Children[0].Type);
      Assert.AreEqual("expressionBody", result.Children[1].Type);
      Assert.AreEqual(1, result.Children[1].Children.Count);
      Assert.AreEqual("id", result.Children[1].Children[0].Type);
      Assert.AreEqual("a", result.Children[1].Children[0].Value);
      Assert.AreEqual("rightParen", result.Children[2].Type);
    }

    [TestMethod]
    public void ParseParseTest()
    {
      Parser parser = new Parser(@"
        a => b ;
      ");

      //Parser.Result result = parser.Parse("(a)");
      //Assert.IsNotNull(result);
    }
  }
}
