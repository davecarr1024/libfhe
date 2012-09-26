using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Reflection;

namespace terp
{
  public abstract class Value
  {
    public Scope ParentScope { get; set; }
  }

  public class NullValue : Value
  {
  }

  public class BoolValue : Value
  {
    public bool Value { get; set; }
  }

  public class IntValue : Value
  {
    public int Value { get; set; }
  }

  public class FloatValue : Value
  {
    public float Value { get; set; }
  }

  public class StringValue : Value
  {
    public string Value { get; set; }
  }

  public class Parameter
  {
    public string Name { get; set; }

    public bool Matches(Value value)
    {
      return true;
    }
  }

  public class Function : Value
  {
    public List<Parameter> Parameters { get; set; }

    public List<Expression> Expressions { get; set; }

    public Function()
    {
      Parameters = new List<Parameter>();
      Expressions = new List<Expression>();
    }

    public bool SignatureMatches(List<Value> args)
    {
      if (args.Count != Parameters.Count)
      {
        return false;
      }
      else
      {
        for (int i = 0; i < args.Count; ++i)
        {
          if (!Parameters[i].Matches(args[i]))
          {
            return false;
          }
        }
        return true;
      }
    }

    public Value Apply(Scope scope, List<Value> args)
    {
      if (args.Count != Parameters.Count)
      {
        throw new Exception("function arg count mismatch: got " + args.Count + " expected " + Parameters.Count);
      }
      else
      {
        Scope newScope = new Scope(scope);
        for (int i = 0; i < args.Count; ++i)
        {
          if (!Parameters[i].Matches(args[i]))
          {
            throw new Exception("function param mismatch: arg " + args[i] + " doesn't satisfy param " + Parameters[i]);
          }
          newScope.SetValue(Parameters[i].Name, args[i]);
        }
        if (Expressions.Any())
        {
          foreach (Expression expression in Expressions.GetRange(0, Expressions.Count - 1))
          {
            expression.Evaluate(newScope);
          }
          Expression last = Expressions.Last();
          if (last != null)
          {
            return last.Evaluate(newScope);
          }
        }
        return new NullValue();
      }
    }
  }

  public class Scope : Value
  {
    public Dictionary<string, Value> Values
    {
      get { return new Dictionary<string, Value>(values); }
      set
      {
        foreach (KeyValuePair<string, Value> item in value)
        {
          SetValue(item.Key, item.Value);
        }
      }
    }

    private Dictionary<string, Value> values;

    public Scope()
    {
      values = new Dictionary<string, Value>();
    }

    public Scope(Scope scope)
    {
      values = new Dictionary<string, Value>(scope.values);
    }

    private void SetValue(List<string> names, Value value)
    {
      if (names.Count == 1)
      {
        values[names.First()] = value;
        value.ParentScope = this;
      }
      else
      {
        Value scope = getValue(names.First());
        if (scope is Scope)
        {
          (scope as Scope).SetValue(names.GetRange(1, names.Count - 1), value);
        }
        else
        {
          throw new Exception("trying to set value into non-scope value " + scope);
        }
      }
    }

    public void SetValue(string name, Value value)
    {
      SetValue(name.Split('.').ToList(), value);
    }

    private Value getValue(string name)
    {
      if (values.ContainsKey(name))
      {
        return values[name];
      }
      else if (ParentScope != null)
      {
        return ParentScope.GetValue(name);
      }
      else
      {
        return null;
      }
    }

    private Value getValue(List<string> names)
    {
      if (names.Count == 1)
      {
        return getValue(names.First());
      }
      else
      {
        Value value = getValue(names.First());
        if (value is Scope)
        {
          return (value as Scope).getValue(names.GetRange(1, names.Count - 1));
        }
        else
        {
          throw new Exception("trying to get value from non-scope value " + value);
        }
      }
    }

    public Value GetValue(string name)
    {
      return getValue(name.Split('.').ToList());
    }
  }

  public class Class : Scope
  {
    public string Name { get; set; }

    public Class Parent { get; set; }

    private void Bind(ClassInstance obj)
    {
      if (Parent != null)
      {
        Parent.Bind(obj);
      }
      foreach (KeyValuePair<string, Value> item in Values)
      {
        if (item.Value is Function)
        {
          Function unboundFunc = item.Value as Function;

          List<Parameter> boundParams = unboundFunc.Parameters.GetRange(1, unboundFunc.Parameters.Count - 1);
          List<Expression> boundArgs = new List<Expression>();
          boundArgs.Add(new Literal() { Value = obj });
          foreach (Parameter param in boundParams)
          {
            boundArgs.Add(new Variable() { Name = param.Name });
          }

          Function boundFunc = new Function()
          {
            Parameters = boundParams,
            Expressions =
            {
              new Invocation()
              {
                Functor = new Literal(){Value = unboundFunc},
                Args = boundArgs
              }
            }
          };

          obj.SetValue(item.Key, boundFunc);
        }
      }
    }

    public ClassInstance Construct(Scope scope, List<Value> args)
    {
      ClassInstance obj = new ClassInstance() { Class = this };

      Bind(obj);

      Function ctor = obj.GetValue("__init__") as Function;
      if (ctor != null)
      {
        ctor.Apply(scope, args);
      }

      return obj;
    }
  }

  public class ClassInstance : Scope
  {
    public Class Class { get; set; }
  }

  public abstract class Expression
  {
    public abstract Value Evaluate(Scope scope);
  }

  public class Assignment : Expression
  {
    public string Name { get; set; }

    public Expression Value { get; set; }

    public override Value Evaluate(Scope scope)
    {
      Value value = Value.Evaluate(scope);
      scope.SetValue(Name, value);
      return value;
    }
  }

  public class Literal : Expression
  {
    public Value Value { get; set; }

    public override Value Evaluate(Scope scope)
    {
      return Value;
    }
  }

  public class Variable : Expression
  {
    public string Name { get; set; }

    public override Value Evaluate(Scope scope)
    {
      return scope.GetValue(Name);
    }
  }

  public class Invocation : Expression
  {
    public Expression Functor { get; set; }

    public List<Expression> Args { get; set; }

    public override Value Evaluate(Scope scope)
    {
      List<Value> args = Args.Select(arg => arg.Evaluate(scope)).ToList();
      Value functorValue = Functor.Evaluate(scope);
      if (functorValue is Function)
      {
        return (functorValue as Function).Apply(scope, args);
      }
      else if (functorValue is Class)
      {
        return (functorValue as Class).Construct(scope, args);
      }
      else if (functorValue is ClassInstance)
      {
        Value callFunction = (functorValue as ClassInstance).GetValue("__call__");
        if (callFunction is Function)
        {
          return (callFunction as Function).Apply(scope, args);
        }
        else
        {
          throw new Exception("trying to invoke class instance " + functorValue + " but it doesn't have a __call__ function");
        }
      }
      else
      {
        throw new Exception("trying to invoke non-functor value " + functorValue);
      }
    }

    public Invocation()
    {
      Args = new List<Expression>();
    }
  }

  [AttributeUsage(AttributeTargets.Method, AllowMultiple = false)]
  public class BuiltinMethod : Attribute { }

  public class BuiltinInt : Value
  {

  }

  public static class VirtualMachine
  {
    public static Scope DefaultScope()
    {
      Scope scope = new Scope();

      foreach (Type type in Assembly.GetExecutingAssembly().GetTypes())
      {
        if (type.Namespace == "terp" && type.GetMethods().Any(method => method.GetCustomAttributes(true).OfType<BuiltinMethod>().Any()))
        {
        }
      }

      return scope;
    }
  }
}