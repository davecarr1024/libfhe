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

    public abstract string GetValueType();
  }

  public class NullValue : Value
  {
    public override string GetValueType()
    {
      return "Null";
    }
  }

  public class Parameter
  {
    public string Name { get; set; }

    public string Type { get; set; }

    public bool Matches(Value value)
    {
      return value.GetValueType() == Type;
    }
  }

  public abstract class IFunction : Value
  {
    public abstract Value Apply(Scope scope, List<Value> args);
  }

  public class Function : IFunction
  {
    public List<Parameter> Parameters { get; set; }

    public List<Expression> Expressions { get; set; }

    public Function()
    {
      Parameters = new List<Parameter>();
      Expressions = new List<Expression>();
    }

    public override Value Apply(Scope scope, List<Value> args)
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

    public override string GetValueType()
    {
      return "Function";
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

    public override string GetValueType()
    {
      return "Scope";
    }
  }

  public abstract class IClass : Scope
  {
    protected abstract void Bind(ClassInstance obj);

    public string Name { get; set; }

    public ClassInstance Construct(Scope scope, List<Value> args)
    {
      ClassInstance obj = new ClassInstance() { Class = this };

      Bind(obj);

      IFunction ctor = obj.GetValue("__init__") as IFunction;
      if (ctor != null)
      {
        ctor.Apply(scope, args);
      }

      return obj;
    }

    public override string GetValueType()
    {
      return Name;
    }
  }

  public class Class : IClass
  {
    public Class Parent { get; set; }

    protected override void Bind(ClassInstance obj)
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
  }

  public class ClassInstance : Scope
  {
    public IClass Class { get; set; }

    public override string GetValueType()
    {
      return Class.Name;
    }
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
      if (functorValue is IFunction)
      {
        return (functorValue as IFunction).Apply(scope, args);
      }
      else if (functorValue is IClass)
      {
        return (functorValue as IClass).Construct(scope, args);
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
  public class BuiltinMethod : Attribute
  {
    public string Name { get; set; }
  }

  public class BuiltinClass : IClass
  {
    protected BuiltinClass(string name)
    {
      Name = name;
    }

    protected override void Bind(ClassInstance obj)
    {
      object builtinObject = GetType().InvokeMember(null, BindingFlags.CreateInstance, null, null, new object[] { });
      foreach (MethodInfo method in GetType().GetMethods())
      {
        if (method.GetCustomAttributes(true).OfType<BuiltinMethod>().Any())
        {
          obj.SetValue(method.Name, new BuiltinFunction() { BuiltinMethod = method, BuiltinObject = builtinObject });
        }
      }
    }
  }

  public class BuiltinFunction : IFunction
  {
    public MethodInfo BuiltinMethod { get; set; }

    public object BuiltinObject { get; set; }

    public override Value Apply(Scope scope, List<Value> unboundArgs)
    {
      List<ParameterInfo> parameters = BuiltinMethod.GetParameters().ToList();

      if (parameters.Count != unboundArgs.Count)
      {
        throw new Exception("unable to call builtin method " + BuiltinMethod.Name + " arg count mismatch");
      }
      else
      {
        List<object> boundArgs = new List<object>();

        for (int i = 0; i < parameters.Count; ++i)
        {
          if (parameters[i].ParameterType.IsAssignableFrom(unboundArgs[i].GetType()))
          {
            boundArgs.Add(unboundArgs[i]);
          }
          else
          {
            throw new Exception("builtin function " + BuiltinMethod.Name + " failed to apply: can't convert arg " + i + " from " + unboundArgs[i].GetType() + " to " + parameters[i].ParameterType);
          }
        }

        Value ret = BuiltinMethod.Invoke(BuiltinObject, boundArgs.ToArray()) as Value;
        if (ret != null)
        {
          return ret;
        }
        else
        {
          return new NullValue();
        }
      }
    }

    public override string GetValueType()
    {
      return "BuiltinFunction";
    }
  }

  public class BuiltinInt : BuiltinClass
  {
    public int Value { get; set; }

    public BuiltinInt()
      : base("int")
    {
    }
  }

  public static class VirtualMachine
  {
    public static List<Type> GetBuiltinClasses()
    {
      List<Type> types = new List<Type>();
      foreach (Type type in Assembly.GetExecutingAssembly().GetTypes())
      {
        if (type.Namespace == "terp" && typeof(BuiltinClass).IsAssignableFrom(type) && type != typeof(BuiltinClass))
        {
          types.Add(type);
        }
      }
      return types;
    }

    public static Scope DefaultScope()
    {
      Scope scope = new Scope();

      foreach (Type type in GetBuiltinClasses())
      {
        BuiltinClass builtinClass = type.InvokeMember(null, BindingFlags.CreateInstance, null, null, new object[0]) as BuiltinClass;
        scope.SetValue(builtinClass.Name, builtinClass);
      }

      return scope;
    }
  }
}