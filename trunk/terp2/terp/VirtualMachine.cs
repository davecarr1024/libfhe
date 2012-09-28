using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Reflection;
using System.Xml.Serialization;
using System.IO;

namespace terp
{
  /// <summary>
  /// Base value
  /// </summary>
  [XmlInclude(typeof(NullValue))]
  [XmlInclude(typeof(IFunction))]
  [XmlInclude(typeof(Scope))]
  public abstract class Value
  {
    /// <summary>
    /// Scope this value is in
    /// </summary>
    [XmlIgnore]
    public Scope ParentScope { get; set; }

    /// <summary>
    /// string repr of this value's type
    /// </summary>
    /// <returns></returns>
    public abstract string GetValueType();

    public void Save(string filename)
    {
      XmlSerializer serializer = new XmlSerializer(typeof(Value));
      TextWriter textWriter = new StreamWriter(filename);
      serializer.Serialize(textWriter, this);
      textWriter.Close();
    }

    public static Value Load(string filename)
    {
      XmlSerializer serializer = new XmlSerializer(typeof(Value));
      TextReader textReader = new StreamReader(filename);
      Value value = serializer.Deserialize(textReader) as Value;
      textReader.Close();
      return value;
    }
  }

  /// <summary>
  /// Null value
  /// </summary>
  public class NullValue : Value
  {
    public override string GetValueType()
    {
      return "Null";
    }
  }

  /// <summary>
  /// Description of a function parameter
  /// </summary>
  public class Parameter
  {
    /// <summary>
    /// Name of the parameter
    /// </summary>
    public string Name { get; set; }

    /// <summary>
    /// Type of the parameter
    /// </summary>
    public string Type { get; set; }

    /// <summary>
    /// Can the given value be bound as this parameter?
    /// </summary>
    /// <param name="value"></param>
    /// <returns></returns>
    public bool Matches(Value value)
    {
      return value.GetValueType() == Type;
    }
  }

  /// <summary>
  /// Base function
  /// </summary>
  [XmlInclude(typeof(Function))]
  [XmlInclude(typeof(FunctionOverload))]
  [XmlInclude(typeof(Method))]
  [XmlInclude(typeof(BuiltinFunction))]
  public abstract class IFunction : Value
  {
    /// <summary>
    /// Name of this function
    /// </summary>
    public string Name { get; set; }

    /// <summary>
    /// Can these args be used with this function?
    /// </summary>
    /// <param name="args"></param>
    /// <returns></returns>
    public abstract bool SignatureMatches(List<Value> args);

    /// <summary>
    /// Run this function with these args in this scope
    /// </summary>
    /// <param name="scope"></param>
    /// <param name="args"></param>
    /// <returns></returns>
    public abstract Value Apply(Scope scope, List<Value> args);
  }

  /// <summary>
  /// An overloaded function
  /// </summary>
  public class FunctionOverload : IFunction
  {
    /// <summary>
    /// All the functions that can be called
    /// </summary>
    public List<IFunction> Functions { get; set; }

    /// <summary>
    /// Can any of the contained function be run with these args?
    /// </summary>
    /// <param name="args"></param>
    /// <returns></returns>
    public override bool SignatureMatches(List<Value> args)
    {
      foreach (IFunction function in Functions)
      {
        if (function.SignatureMatches(args))
        {
          return true;
        }
      }
      return false;
    }

    /// <summary>
    /// Try to run one of the functions with these args in this scope
    /// </summary>
    /// <param name="scope"></param>
    /// <param name="args"></param>
    /// <returns></returns>
    public override Value Apply(Scope scope, List<Value> args)
    {
      foreach (IFunction function in Functions)
      {
        if (function.SignatureMatches(args))
        {
          return function.Apply(new Scope(scope), args);
        }
      }
      throw new Exception("Unable to find function overload to match args " + args);
    }

    public FunctionOverload()
    {
      Functions = new List<IFunction>();
    }

    public override string GetValueType()
    {
      return "FunctionOverload";
    }
  }

  /// <summary>
  /// Basic function
  /// </summary>
  public class Function : IFunction
  {
    /// <summary>
    /// Parameters this function accepts
    /// </summary>
    public List<Parameter> Parameters { get; set; }

    /// <summary>
    /// Expressions that compose this function's contents
    /// </summary>
    public List<Expression> Expressions { get; set; }

    public Function()
    {
      Parameters = new List<Parameter>();
      Expressions = new List<Expression>();
    }

    /// <summary>
    /// Can this function be run with these args
    /// </summary>
    /// <param name="args"></param>
    /// <returns></returns>
    public override bool SignatureMatches(List<Value> args)
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

    /// <summary>
    /// Run this function with these args in this scope
    /// </summary>
    /// <param name="scope"></param>
    /// <param name="args"></param>
    /// <returns></returns>
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
          return last.Evaluate(newScope);
        }
        else
        {
          return new NullValue();
        }
      }
    }

    public override string GetValueType()
    {
      return "Function";
    }
  }

  /// <summary>
  /// A value that holds other named values
  /// </summary>
  [XmlInclude(typeof(IClass))]
  [XmlInclude(typeof(Function))]
  [XmlInclude(typeof(Function))]
  public class Scope : Value
  {
    public class ScopeValue
    {
      public string Name { get; set; }
      public Value Value { get; set; }
    }

    /// <summary>
    /// The values contained in this scope
    /// note: don't call Values.add, do Values = new Dictionary or call SetValue()
    /// </summary>
    public List<ScopeValue> Values
    {
      get { return values.Select(item => new ScopeValue() { Name = item.Key, Value = item.Value }).ToList(); }
      set { value.ForEach(item => SetValue(item.Name, item.Value)); }
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

    /// <summary>
    /// Implementation of SetValue after splitting out dot operators
    /// </summary>
    /// <param name="names"></param>
    /// <param name="value"></param>
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

    /// <summary>
    /// Set a value into this scope, overloading function if necessary and derefing dot operators
    /// </summary>
    /// <param name="name"></param>
    /// <param name="value"></param>
    public void SetValue(string name, Value value)
    {
      if (value is IFunction)
      {
        Value currentValue = GetValue(name);
        if (currentValue is FunctionOverload)
        {
          (currentValue as FunctionOverload).Functions.Add(value as IFunction);
        }
        else if (currentValue is IFunction)
        {
          SetValue(name.Split('.').ToList(), new FunctionOverload() { Functions = { currentValue as IFunction, value as IFunction } });
        }
        else
        {
          SetValue(name.Split('.').ToList(), value);
        }
      }
      else
      {
        SetValue(name.Split('.').ToList(), value);
      }
    }

    /// <summary>
    /// Base getValue operation
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
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

    /// <summary>
    /// getValue after dot operator split
    /// </summary>
    /// <param name="names"></param>
    /// <returns></returns>
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

    /// <summary>
    /// Get a value with a given name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    public Value GetValue(string name)
    {
      return getValue(name.Split('.').ToList());
    }

    public override string GetValueType()
    {
      return "Scope";
    }
  }

  /// <summary>
  /// Base class: different from a scope because it is named and can be instantiated
  /// </summary>
  [XmlInclude(typeof(Class))]
  [XmlInclude(typeof(BuiltinClass))]
  public abstract class IClass : Scope
  {
    /// <summary>
    /// Add all class properties to the given instance
    /// </summary>
    /// <param name="obj"></param>
    public abstract void Bind(ClassInstance obj);

    /// <summary>
    /// Name of this class
    /// </summary>
    public string Name { get; set; }

    /// <summary>
    /// Build a class instance out of this class, call __init__ if found
    /// </summary>
    /// <param name="scope"></param>
    /// <param name="args"></param>
    /// <returns></returns>
    public ClassInstance Construct(Scope scope, List<Value> args)
    {
      ClassInstance obj = new ClassInstance() { Class = this };

      Bind(obj);

      FunctionOverload ctor = obj.GetValue("__init__") as FunctionOverload;
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

  /// <summary>
  /// A function that has been bound to a class instance
  /// </summary>
  public class Method : IFunction
  {
    /// <summary>
    /// The class instance this function is bound to
    /// </summary>
    public ClassInstance ClassInstance { get; set; }

    /// <summary>
    /// The function this is bound to
    /// </summary>
    public IFunction Function { get; set; }

    /// <summary>
    /// Handle adding implicit this as first arg to unbound function
    /// </summary>
    /// <param name="unboundArgs"></param>
    /// <returns></returns>
    private List<Value> BindArgs(List<Value> unboundArgs)
    {
      List<Value> boundArgs = new List<Value>();
      boundArgs.Add(ClassInstance);
      boundArgs.AddRange(unboundArgs);
      return boundArgs;
    }

    /// <summary>
    /// Can this be called with the given args? 
    /// </summary>
    /// <param name="args"></param>
    /// <returns></returns>
    public override bool SignatureMatches(List<Value> args)
    {
      return Function.SignatureMatches(BindArgs(args));
    }

    /// <summary>
    /// Run this function with the given args
    /// </summary>
    /// <param name="scope"></param>
    /// <param name="args"></param>
    /// <returns></returns>
    public override Value Apply(Scope scope, List<Value> args)
    {
      return Function.Apply(scope, BindArgs(args));
    }

    public override string GetValueType()
    {
      return "Method";
    }
  }

  /// <summary>
  /// Basic class
  /// </summary>
  public class Class : IClass
  {
    /// <summary>
    /// The class this inherits from, if any
    /// </summary>
    public Class Parent { get; set; }

    /// <summary>
    /// Bind the instance with this class's methods
    /// </summary>
    /// <param name="obj"></param>
    public override void Bind(ClassInstance obj)
    {
      if (Parent != null)
      {
        Parent.Bind(obj);
      }
      foreach (Scope.ScopeValue item in Values)
      {
        if (item.Value is IFunction)
        {
          obj.SetValue(item.Name, new Method() { ClassInstance = obj, Function = item.Value as IFunction });
        }
      }
    }
  }

  /// <summary>
  /// An instance of a class
  /// </summary>
  public class ClassInstance : Scope
  {
    /// <summary>
    /// The class that was instantiated to make this
    /// </summary>
    [XmlIgnore]
    public IClass Class { get; set; }

    public string ClassName
    {
      get { return Class.Name; }
      set
      {
        Class = GetValue(value) as IClass;
      }
    }

    /// <summary>
    /// The builtin instance that this represents, if any
    /// </summary>
    public Builtin BuiltinInstance { get; set; }

    public override string GetValueType()
    {
      return Class.Name;
    }

    public override string ToString()
    {
      return GetValueType();
    }
  }

  /// <summary>
  /// Mark a method as a builtin function to be exposed
  /// </summary>
  [AttributeUsage(AttributeTargets.Method, AllowMultiple = false)]
  public class BuiltinFunctionAttribute : Attribute
  {
  }

  /// <summary>
  /// Mark a class as a builtin class to be exposed
  /// </summary>
  [AttributeUsage(AttributeTargets.Class, AllowMultiple = false)]
  public class BuiltinClassAttribute : Attribute
  {
    public string Name { get; set; }
  }

  /// <summary>
  /// Abstract type representing a builtin class
  /// </summary>
  public class BuiltinClass : IClass
  {
    /// <summary>
    /// Builtin type that is being represented
    /// </summary>
    [XmlIgnore]
    public Type Type
    {
      get { return type; }
      set
      {
        type = value;
        BuiltinClassAttribute attr = type.GetCustomAttributes(false).OfType<BuiltinClassAttribute>().FirstOrDefault();
        if (attr != null && !string.IsNullOrEmpty(attr.Name))
        {
          Name = attr.Name;
        }
        else
        {
          Name = type.Name;
        }
      }
    }

    private Type type;

    /// <summary>
    /// Bind all the c# methods in the builtin class to the new instance, construct a new builtin class if necessary
    /// </summary>
    /// <param name="obj"></param>
    /// <param name="builtinInstance"></param>
    private void Bind(ClassInstance obj, Builtin builtinInstance)
    {
      if (builtinInstance == null)
      {
        builtinInstance = Type.InvokeMember(null, BindingFlags.CreateInstance, null, null, new object[] { }) as Builtin;
      }
      obj.BuiltinInstance = builtinInstance;
      foreach (MethodInfo method in Type.GetMethods())
      {
        if (method.GetCustomAttributes(true).OfType<BuiltinFunctionAttribute>().Any())
        {
          obj.SetValue(method.Name, new BuiltinFunction() { BuiltinMethod = method, BuiltinInstance = obj.BuiltinInstance });
        }
      }
    }

    /// <summary>
    /// Bind a new object
    /// </summary>
    /// <param name="obj"></param>
    public override void Bind(ClassInstance obj)
    {
      Bind(obj, null);
    }

    /// <summary>
    /// Construct a Value from a given builtin instance
    /// </summary>
    /// <param name="builtinType"></param>
    /// <param name="builtinInstance"></param>
    /// <returns></returns>
    private static ClassInstance Construct(Type builtinType, Builtin builtinInstance)
    {
      if (!VirtualMachine.BuiltinClasses.ContainsKey(builtinType))
      {
        throw new Exception("unable to construct unknown builtin type " + builtinType);
      }
      BuiltinClass builtinClass = VirtualMachine.BuiltinClasses[builtinType];
      ClassInstance obj = new ClassInstance() { Class = builtinClass };
      builtinClass.Bind(obj, builtinInstance);
      return obj;
    }

    /// <summary>
    /// Construct a Value from a given builtin instance
    /// </summary>
    /// <param name="builtinInstance"></param>
    /// <returns></returns>
    public static ClassInstance Construct(Builtin builtinInstance)
    {
      return Construct(builtinInstance.GetType(), builtinInstance);
    }

    /// <summary>
    /// Construct a Value from a given builtin type
    /// </summary>
    /// <param name="builtinType"></param>
    /// <returns></returns>
    public static ClassInstance Construct(Type builtinType)
    {
      return Construct(builtinType, null);
    }
  }

  /// <summary>
  /// A function bound to a builtin method
  /// </summary>
  public class BuiltinFunction : IFunction
  {
    /// <summary>
    /// The method this function represents
    /// </summary>
    [XmlIgnore]
    public MethodInfo BuiltinMethod { get; set; }

    /// <summary>
    /// The object that this function is bound to
    /// </summary>
    [XmlIgnore]
    public Builtin BuiltinInstance { get; set; }

    /// <summary>
    /// Can this function be called with these args
    /// </summary>
    /// <param name="args"></param>
    /// <returns></returns>
    public override bool SignatureMatches(List<Value> args)
    {
      List<ParameterInfo> parameters = BuiltinMethod.GetParameters().ToList();

      if (parameters.Count != args.Count)
      {
        return false;
      }
      else
      {
        List<object> boundArgs = new List<object>();

        for (int i = 0; i < parameters.Count; ++i)
        {
          if (!parameters[i].ParameterType.IsAssignableFrom(args[i].GetType()))
          {
            return false;
          }
        }
        return true;
      }
    }

    /// <summary>
    /// Call this function with these args
    /// </summary>
    /// <param name="scope"></param>
    /// <param name="unboundArgs"></param>
    /// <returns></returns>
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
          else if (unboundArgs[i] is ClassInstance &&
                   (unboundArgs[i] as ClassInstance).BuiltinInstance != null &&
                   parameters[i].ParameterType.IsAssignableFrom((unboundArgs[i] as ClassInstance).BuiltinInstance.GetType()))
          {
            boundArgs.Add((unboundArgs[i] as ClassInstance).BuiltinInstance);
          }
          else
          {
            throw new Exception("builtin function " + BuiltinMethod.Name + " failed to apply: can't convert arg " + i + " from " + unboundArgs[i].GetType() + " to " + parameters[i].ParameterType);
          }
        }

        object ret = BuiltinMethod.Invoke(BuiltinInstance, boundArgs.ToArray());
        if (ret is Value)
        {
          return ret as Value;
        }
        else if (ret is Builtin && VirtualMachine.BuiltinClasses.ContainsKey(ret.GetType()))
        {
          ClassInstance obj = BuiltinClass.Construct(ret as Builtin);
          return obj;
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

  [XmlInclude(typeof(BuiltinInt))]
  public abstract class Builtin
  {
  }

  /// <summary>
  /// Builtin int type
  /// </summary>
  [BuiltinClass(Name = "int")]
  public class BuiltinInt : Builtin
  {
    public int Value { get; set; }

    /// <summary>
    /// + operator
    /// </summary>
    /// <param name="i"></param>
    /// <returns></returns>
    [BuiltinFunction]
    public BuiltinInt __add__(BuiltinInt i)
    {
      return new BuiltinInt() { Value = i.Value + Value };
    }

    /// <summary>
    /// Is the given Value a builtinInt with value i?
    /// </summary>
    /// <param name="value"></param>
    /// <param name="i"></param>
    /// <returns></returns>
    public static bool ValueEquals(Value value, int i)
    {
      return value is ClassInstance &&
             (value as ClassInstance).BuiltinInstance is BuiltinInt &&
             ((value as ClassInstance).BuiltinInstance as BuiltinInt).Value == i;
    }

    /// <summary>
    /// Build a new Value that is an instance of a Builtinint with value i
    /// </summary>
    /// <param name="i"></param>
    /// <returns></returns>
    public static Value Construct(int i)
    {
      return BuiltinClass.Construct(new BuiltinInt() { Value = i });
    }
  }

  /// <summary>
  /// Abstract expression
  /// </summary>
  [XmlInclude(typeof(Variable))]
  [XmlInclude(typeof(Invocation))]
  [XmlInclude(typeof(Assignment))]
  [XmlInclude(typeof(BinaryOperation))]
  [XmlInclude(typeof(Literal))]
  public abstract class Expression
  {
    public abstract Value Evaluate(Scope scope);
  }

  /// <summary>
  /// Assign a value to a name in a scope
  /// </summary>
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

  public enum BinaryOperator
  {
    Add,
    Subtract,
    Multiply,
    Divide,
    And,
    Or,
  }

  /// <summary>
  /// Apply a binary operator to two operands
  /// </summary>
  public class BinaryOperation : Expression
  {
    public BinaryOperator Operator { get; set; }

    public Expression LeftOperand { get; set; }

    public Expression RightOperand { get; set; }

    public static string GetOperatorMethodName(BinaryOperator op)
    {
      switch (op)
      {
        case BinaryOperator.Add:
          return "__add__";
        case BinaryOperator.Subtract:
          return "__sub__";
        case BinaryOperator.Multiply:
          return "__mul__";
        case BinaryOperator.Divide:
          return "__div__";
        case BinaryOperator.And:
          return "__and__";
        case BinaryOperator.Or:
          return "__or__";
        default:
          throw new NotImplementedException();
      }
    }

    public override Value Evaluate(Scope scope)
    {
      string methodName = GetOperatorMethodName(Operator);
      Value leftOperand = LeftOperand.Evaluate(scope);
      Value rightOperand = RightOperand.Evaluate(scope);
      Scope leftScope = leftOperand as Scope;
      if (leftScope != null)
      {
        IFunction leftFunc = leftScope.GetValue(methodName) as IFunction;
        if (leftFunc != null)
        {
          return leftFunc.Apply(scope, new List<Value>() { rightOperand });
        }
      }
      throw new Exception("failed to apply binary operator " + Operator + " to operands " + leftOperand + " and " + rightOperand);
    }
  }

  /// <summary>
  /// Bind a Value to an Expression
  /// </summary>
  public class Literal : Expression
  {
    public Value Value { get; set; }

    public override Value Evaluate(Scope scope)
    {
      return Value;
    }
  }

  /// <summary>
  /// Bind a scope member to an Expression
  /// </summary>
  public class Variable : Expression
  {
    public string Name { get; set; }

    public override Value Evaluate(Scope scope)
    {
      Value value = scope.GetValue(Name);
      if (value == null)
      {
        throw new Exception("unknown variable " + Name);
      }
      return value;
    }
  }

  /// <summary>
  /// Invoke a function, class, or object.__call__ with some args
  /// </summary>
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
        if (callFunction is IFunction)
        {
          return (callFunction as IFunction).Apply(scope, args);
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

  /// <summary>
  /// Overall virtual machine state
  /// </summary>
  public static class VirtualMachine
  {
    static VirtualMachine()
    {
      BuiltinClasses = new Dictionary<Type, BuiltinClass>();
      foreach (Type type in Assembly.GetExecutingAssembly().GetTypes())
      {
        if (type.GetCustomAttributes(false).Any(attr => attr is BuiltinClassAttribute))
        {
          BuiltinClasses[type] = new BuiltinClass() { Type = type };
        }
      }
    }

    public static Dictionary<Type, BuiltinClass> BuiltinClasses { get; set; }

    public static Scope DefaultScope()
    {
      Scope scope = new Scope();

      foreach (BuiltinClass builtinClass in BuiltinClasses.Values)
      {
        scope.SetValue(builtinClass.Name, builtinClass);
      }

      return scope;
    }
  }
}
