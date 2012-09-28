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
    public void builtin()
    {
      /**
       * a = int();
       * assert( system.type( a ) == "int" );
       */
      Scope scope = VirtualMachine.DefaultScope();
      ClassInstance i = new Invocation() { Functor = new Variable() { Name = "int" } }.Evaluate(scope) as ClassInstance;
      Assert.IsNotNull(i);
      Assert.AreEqual("int", i.GetValueType());
    }

    [TestMethod]
    public void userClass()
    {
      /*
       * class foo { function bar( foo this ) { 1; } }
       * fooinstance = foo();
       * result = foo.bar();
       * assert( result == 1 );
       */
      Scope scope = VirtualMachine.DefaultScope();
      new Assignment()
      {
        Name = "foo",
        Value = new Literal()
        {
          Value = new Class()
          {
            Name = "foo",
            Values = new List<Scope.ScopeValue>()
            {
              new Scope.ScopeValue()
              {
                Name = "bar", 
                Value = new Function()
                {
                  Parameters = { new Parameter() { Name = "this", Type = "foo" } },
                  Expressions = { new Literal() { Value = BuiltinInt.Construct(1) } }
                }
              }
            }
          }
        }
      }.Evaluate(scope);
      new Assignment() { Name = "fooinstance", Value = new Invocation() { Functor = new Variable() { Name = "foo" } } }.Evaluate(scope);

      Class foo = scope.GetValue("foo") as Class;
      Assert.IsNotNull(foo);
      ClassInstance fooinstance = scope.GetValue("fooinstance") as ClassInstance;
      Assert.IsNotNull(fooinstance);
      Assert.AreEqual(foo.Values.Count, fooinstance.Values.Count);

      new Assignment() { Name = "result", Value = new Invocation() { Functor = new Variable() { Name = "fooinstance.bar" } } }.Evaluate(scope);
      Assert.IsTrue(BuiltinInt.ValueEquals(scope.GetValue("result"), 1));
    }

    [TestMethod]
    public void overload()
    {
      /*
       * class foo 
       * { 
       *   function bar( foo this ) { 1; } 
       *   function bar( foo this, int i ) { i; } 
       * }
       * fooinstance = foo();
       * result = foo.bar();
       * assert( result == 1 );
       * result = foo.bar( 2 );
       * assert( result == 2 );
       */
      Scope scope = VirtualMachine.DefaultScope();
      new Assignment()
      {
        Name = "foo",
        Value = new Literal()
        {
          Value = new Class()
          {
            Name = "foo",
            Values = new List<Scope.ScopeValue>()
            {
              new Scope.ScopeValue()
              {
                Name = "bar", 
                Value = new Function()
                {
                  Parameters = { new Parameter() { Name = "this", Type = "foo" } },
                  Expressions = { new Literal() { Value = BuiltinInt.Construct(1) } }
                }
              }
            }
          }
        }
      }.Evaluate(scope);
      new Assignment()
      {
        Name = "foo.bar",
        Value = new Literal()
        {
          Value = new Function()
          {
            Parameters = { 
              new Parameter() { Name = "this", Type = "foo" },
              new Parameter(){ Name = "i", Type = "int" },
            },
            Expressions = { new Variable() { Name = "i" } }
          }
        }
      }.Evaluate(scope);
      new Assignment() { Name = "fooinstance", Value = new Invocation() { Functor = new Variable() { Name = "foo" } } }.Evaluate(scope);

      Class foo = scope.GetValue("foo") as Class;
      Assert.IsNotNull(foo);
      ClassInstance fooinstance = scope.GetValue("fooinstance") as ClassInstance;
      Assert.IsNotNull(fooinstance);
      Assert.AreEqual(foo.Values.Count, fooinstance.Values.Count);

      new Assignment() { Name = "result", Value = new Invocation() { Functor = new Variable() { Name = "fooinstance.bar" } } }.Evaluate(scope);
      Assert.IsTrue(BuiltinInt.ValueEquals(scope.GetValue("result"), 1));

      new Assignment()
      {
        Name = "result",
        Value = new Invocation()
        {
          Functor = new Variable() { Name = "fooinstance.bar" },
          Args = { new Literal() { Value = BuiltinInt.Construct(2) } }
        }
      }.Evaluate(scope);
      Assert.IsTrue(BuiltinInt.ValueEquals(scope.GetValue("result"), 2));
    }

    [TestMethod]
    public void operators()
    {
      /*
       * a = 1 + 2;
       * assert( a == 3 );
       */
      Scope scope = VirtualMachine.DefaultScope();
      new Assignment()
      {
        Name = "a",
        Value = new BinaryOperation()
        {
          Operator = BinaryOperator.Add,
          LeftOperand = new Literal() { Value = BuiltinInt.Construct(1) },
          RightOperand = new Literal() { Value = BuiltinInt.Construct(2) }
        }
      }.Evaluate(scope);
      Assert.IsTrue(BuiltinInt.ValueEquals(scope.GetValue("a"), 3));
    }

    [TestMethod]
    public void funcxml()
    {
      /*
       * function func( int a ) { return 5 + a; }
       * system.save( func, "test.xml" );
       * test = system.load( "test.xml" );
       * assert( 8 == func( 3 ) );
       */

      new Function()
      {
        Name = "func",
        Parameters =
        {
          new Parameter() { Type = "int", Name = "a" }
        },
        Expressions =
        {
          new BinaryOperation()
          {
            Operator = BinaryOperator.Add,
            LeftOperand = new Literal(){Value = BuiltinInt.Construct(5)},
            RightOperand = new Variable(){Name = "a"},
          }
        }
      }.Save("test.xml");

      IFunction func = Value.Load("test.xml") as IFunction;
      Assert.IsNotNull(func);

      Assert.IsTrue(BuiltinInt.ValueEquals(func.Apply(VirtualMachine.DefaultScope(), new List<Value>() { BuiltinInt.Construct(3) }), 8));
    }
  }
}
