using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.ObjectModel;
using System.Xml.Serialization;

namespace terp
{
    public class VirtualMachine
    {
        public interface Value
        {
        }

        public class Null : Value
        {
        }

        public class Bool : Value
        {
            public bool Value { get; set; }
        }

        public class Int : Value
        {
            public int Value { get; set; }
        }

        public class Float : Value
        {
            public float Value { get; set; }
        }

        public class String : Value
        {
            public string Value { get; set; }
        }

        public class Assembly : Value
        {
            public ObservableCollection<Namespace> Namespaces
            {
                get { return namespaces; }
                set
                {
                    namespaces = value;
                    namespaces.CollectionChanged += new System.Collections.Specialized.NotifyCollectionChangedEventHandler(namespaces_CollectionChanged);
                    namespaces_CollectionChanged(null, null);
                }
            }

            void namespaces_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
            {
                foreach (Namespace ns in Namespaces.ToList())
                {
                    ns.ParentAssembly = this;
                }
            }

            private ObservableCollection<Namespace> namespaces;

            public Assembly()
            {
                Namespaces = new ObservableCollection<Namespace>();
            }

            public Class FindClass(string name)
            {
                foreach (Namespace ns in Namespaces)
                {
                    Class cl = ns.FindClass(name);
                    if (cl != null)
                    {
                        return cl;
                    }
                }
                return null;
            }
        }

        public class Namespace : Value
        {
            [XmlIgnore]
            public Assembly ParentAssembly
            {
                get { return parentAssembly; }
                set
                {
                    if (value != parentAssembly)
                    {
                        if (parentAssembly != null && parentAssembly.Namespaces.Contains(this))
                        {
                            parentAssembly.Namespaces.Remove(this);
                        }
                        parentAssembly = value;
                        if (parentAssembly != null && !parentAssembly.Namespaces.Contains(this))
                        {
                            parentAssembly.Namespaces.Add(this);
                        }

                    }
                    ParentNamespace = null;
                }
            }

            private Assembly parentAssembly;

            [XmlIgnore]
            public Namespace ParentNamespace
            {
                get { return parentNamespace; }
                set
                {
                    if (parentNamespace != value)
                    {
                        if (parentNamespace != null && parentNamespace.Children.Contains(this))
                        {
                            parentNamespace.Children.Remove(this);
                        }
                        parentNamespace = value;
                        if (parentNamespace != null && !parentNamespace.Children.Contains(this))
                        {
                            parentNamespace.Children.Add(this);
                        }
                    }
                }
            }

            private Namespace parentNamespace;

            public ObservableCollection<Namespace> Children
            {
                get { return children; }
                set
                {
                    children = value;
                    children.CollectionChanged += new System.Collections.Specialized.NotifyCollectionChangedEventHandler(children_CollectionChanged);
                    children_CollectionChanged(null, null);
                }
            }

            void children_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
            {
                foreach (Namespace child in Children.ToList())
                {
                    child.ParentNamespace = this;
                }
            }

            private ObservableCollection<Namespace> children;

            public ObservableCollection<Class> Classes
            {
                get { return classes; }
                set
                {
                    classes = value;
                    classes.CollectionChanged += new System.Collections.Specialized.NotifyCollectionChangedEventHandler(classes_CollectionChanged);
                    classes_CollectionChanged(null, null);
                }
            }

            void classes_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
            {
                foreach (Class cl in Classes.ToList())
                {
                    cl.ParentNamespace = this;
                }
            }

            private ObservableCollection<Class> classes;

            public string Name { get; set; }

            public Namespace()
            {
                Children = new ObservableCollection<Namespace>();
                Classes = new ObservableCollection<Class>();
            }

            public Class FindClass(string name)
            {
                foreach (Class cl in Classes)
                {
                    if (cl.Name == name)
                    {
                        return cl;
                    }
                }
                foreach (Namespace child in Children)
                {
                    Class cl = child.FindClass(name);
                    if (cl != null)
                    {
                        return cl;
                    }
                }
                return null;
            }
        }

        public class Class : Value
        {
            [XmlIgnore]
            public Namespace ParentNamespace
            {
                get { return parentNamespace; }
                set
                {
                    if (parentNamespace != value)
                    {
                        if (parentNamespace != null && parentNamespace.Classes.Contains(this))
                        {
                            parentNamespace.Classes.Remove(this);
                        }
                        parentNamespace = value;
                        if (parentNamespace != null && !parentNamespace.Classes.Contains(this))
                        {
                            parentNamespace.Classes.Add(this);
                        }
                    }
                }
            }

            private Namespace parentNamespace;

            public ObservableCollection<Method> Methods
            {
                get { return methods; }
                set
                {
                    methods = value;
                    methods.CollectionChanged += new System.Collections.Specialized.NotifyCollectionChangedEventHandler(methods_CollectionChanged);
                    methods_CollectionChanged(null, null);
                }
            }

            void methods_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
            {
                foreach (Method method in Methods.ToList())
                {
                    method.ParentClass = this;
                }
            }

            private ObservableCollection<Method> methods;

            public string Name { get; set; }

            public Class()
            {
                Methods = new ObservableCollection<Method>();
            }

            public Object Construct(Context context, List<Value> args)
            {
                Object obj = new Object() { Class = this };
                obj.Methods = new ObservableCollection<BoundMethod>(Methods.Select(method => new BoundMethod() { Object = obj, Method = method }));

                BoundMethod ctor = obj.FindMethod(Name, args);
                if (ctor != null)
                {
                    ctor.Execute(context, args);
                }
                else if (args.Any())
                {
                    throw new Exception("unable to find constructor in class " + Name + " that takes args " + args);
                }

                return obj;
            }
        }

        public class Method : Value
        {
            [XmlIgnore]
            public Class ParentClass
            {
                get { return parentClass; }
                set
                {
                    if (parentClass != value)
                    {
                        if (parentClass != null && parentClass.Methods.Contains(this))
                        {
                            parentClass.Methods.Remove(this);
                        }
                        parentClass = value;
                        if (parentClass != null && !parentClass.Methods.Contains(this))
                        {
                            parentClass.Methods.Add(this);
                        }
                    }
                }
            }

            private Class parentClass;

            public ObservableCollection<Statement> Statements
            {
                get { return statements; }
                set
                {
                    statements = value;
                    statements.CollectionChanged += new System.Collections.Specialized.NotifyCollectionChangedEventHandler(statements_CollectionChanged);
                    statements_CollectionChanged(null, null);
                }
            }

            void statements_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
            {
                foreach (Statement statement in Statements.ToList())
                {
                    statement.ParentMethod = this;
                }
            }

            private ObservableCollection<Statement> statements;

            public List<string> ArgNames { get; set; }

            public string Name { get; set; }

            public Method()
            {
                Statements = new ObservableCollection<Statement>();
                ArgNames = new List<string>();
            }

            public bool SignatureMatches(List<Value> args)
            {
                return args.Count == ArgNames.Count;
            }
        }

        public class Object : Value
        {
            [XmlIgnore]
            public Class Class { get; set; }

            public ObservableCollection<BoundMethod> Methods
            {
                get { return methods; }
                set
                {
                    methods = value;
                }
            }

            private ObservableCollection<BoundMethod> methods;

            public Object()
            {
                Methods = new ObservableCollection<BoundMethod>();
            }

            public List<Value> BindArgs(List<Value> unboundArgs)
            {
                List<Value> boundArgs = new List<Value>();
                boundArgs.Add(this);
                boundArgs.AddRange(unboundArgs);
                return boundArgs;
            }

            public BoundMethod FindMethod(string name, List<Value> unboundArgs)
            {
                List<Value> boundArgs = BindArgs(unboundArgs);
                foreach (BoundMethod method in Methods)
                {
                    if (method.Method.Name == name && method.Method.SignatureMatches(boundArgs))
                    {
                        return method;
                    }
                }
                return null;
            }

            public Value RunMethod(Context context, string name, List<Value> unboundArgs)
            {
                BoundMethod method = FindMethod(name, unboundArgs);
                if (method == null)
                {
                    throw new Exception("unable to find method " + name + " that matches args " + unboundArgs);
                }
                List<Value> boundArgs = BindArgs(unboundArgs);
                return method.Execute(context, boundArgs);
            }
        }

        public class BoundMethod : Value
        {
            [XmlIgnore]
            public Object Object { get; set; }

            [XmlIgnore]
            public Method Method { get; set; }

            public Value Execute(Context parentContext, List<Value> unboundArgs)
            {
                List<Value> boundArgs = Object.BindArgs(unboundArgs);
                if (!Method.SignatureMatches(boundArgs))
                {
                    throw new Exception("unable to run method " + Method.Name + ": invalid args");
                }
                else
                {
                    Context context = new Context(parentContext);
                    for (int i = 0; i < boundArgs.Count; ++i)
                    {
                        context.Stack.Peek().Values[Method.ArgNames[i]] = boundArgs[i];
                    }
                    foreach (Statement statement in Method.Statements)
                    {
                        statement.Execute(context);
                    }
                    return new Null();
                }
            }
        }

        public abstract class Statement
        {
            [XmlIgnore]
            public Method ParentMethod
            {
                get { return parentMethod; }
                set
                {
                    if (parentMethod != value)
                    {
                        if (parentMethod != null && parentMethod.Statements.Contains(this))
                        {
                            parentMethod.Statements.Remove(this);
                        }
                        parentMethod = value;
                        if (parentMethod != null && !parentMethod.Statements.Contains(this))
                        {
                            parentMethod.Statements.Add(this);
                        }
                    }
                }
            }

            private Method parentMethod;

            public abstract void Execute(Context context);
        }

        public class Assignment : Statement
        {
            public string Name { get; set; }

            public Expression Value { get; set; }

            public override void Execute(Context context)
            {
                context.Stack.Peek().Values[Name] = Value.Evaluate(context);
            }
        }

        public abstract class Expression : Statement
        {
            public override void Execute(Context context)
            {
                Evaluate(context);
            }

            public abstract Value Evaluate(Context context);
        }

        public class Literal : Expression
        {
            public Value Value { get; set; }

            public override Value Evaluate(Context context)
            {
                return Value;
            }
        }

        public class Variable : Expression
        {
            public string Name { get; set; }

            public override Value Evaluate(Context context)
            {
                return context.GetValue(Name);
            }
        }

        public class Scope
        {
            public Dictionary<string, Value> Values { get; set; }

            public Scope()
            {
                Values = new Dictionary<string, Value>();
            }

            public Scope(Scope scope)
            {
                Values = new Dictionary<string, Value>(scope.Values);
            }
        }

        public class Context
        {
            public Stack<Scope> Stack { get; set; }

            public Context(Scope scope)
            {
                Stack = new Stack<Scope>();
                Stack.Push(scope);
            }

            public Context(Context context)
            {
                Stack = new Stack<Scope>(context.Stack.Select(scope => new Scope(scope)));
                Stack.Push(new Scope(Stack.Peek()));
            }

            public Value GetValue(string name)
            {
                foreach (Scope scope in Stack)
                {
                    if (scope.Values.ContainsKey(name))
                    {
                        return scope.Values[name];
                    }
                }
                throw new Exception("unknown variable " + name);
            }
        }

        static public Value Execute(Assembly assembly)
        {
            Context context = new Context(new Scope());
            Class programClass = assembly.FindClass("Program");
            if (programClass == null)
            {
                throw new Exception("executable assemblies must have Program class with Main method");
            }
            else
            {
                List<Value> ctorArgs = new List<Value>();
                Object program = programClass.Construct(context, ctorArgs);

                List<Value> mainArgs = new List<Value>();
                BoundMethod main = program.FindMethod("Main", mainArgs);
                return main.Execute(context, mainArgs);
            }
        }
    }
}
