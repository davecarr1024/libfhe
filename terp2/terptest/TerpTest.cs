using System;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using terp;

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

    [TestMethod]
    public void assigment()
    {
      Scope scope = VirtualMachine.DefaultScope();
      new Assignment() { Name = "a", Value = new Literal() { Value = new IntValue() { Value = 12 } } }.Evaluate(scope);
      IntValue a = scope.GetValue("a") as IntValue;
      Assert.IsNotNull(a);
      Assert.AreEqual(12, a.Value);
    }

    [TestMethod]
    public void functionReturn()
    {
      Scope scope = VirtualMachine.DefaultScope();
      StringValue ret = new Function() { Expressions = { new Literal() { Value = new StringValue() { Value = "herro" } } } }.Apply(scope, new List<Value>()) as StringValue;
      Assert.IsNotNull(ret);
      Assert.AreEqual("herro", ret.Value);
    }

    [TestMethod]
    public void functionArgs()
    {
      Scope scope = VirtualMachine.DefaultScope();
      IntValue ret = new Function()
      {
        Parameters = { new Parameter() { Name = "foo" } },
        Expressions = { new Variable() { Name = "foo" } }
      }.Apply(scope, new List<Value>() { new IntValue() { Value = -1 } }) as IntValue;
      Assert.IsNotNull(ret);
      Assert.AreEqual(-1, ret.Value);
    }

    [TestMethod]
    public void ctor()
    {
      Scope scope = VirtualMachine.DefaultScope();
      scope.SetValue("foo", new Class() { Name = "foo", });
      scope.SetValue("foo.__init__",
        new Function()
        {
          Parameters = { new Parameter() { Name = "this" } },
          Expressions = { new Assignment() { Name = "this.a", Value = new Literal() { Value = new IntValue() { Value = 11 } } } }
        }
      );
      new Assignment(){Name = "bar", Value = new Invocation() { Functor = new Variable() { Name = "foo" } }}.Evaluate(scope);
      IntValue a = scope.GetValue("bar.a") as IntValue;
      Assert.IsNotNull(a);
      Assert.AreEqual(11, a.Value);
      Assert.IsNull(scope.GetValue("foo.a"));
    }
  }
}
