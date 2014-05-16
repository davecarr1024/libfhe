﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

namespace Sharpy.Interpreter.Vals
{
    public class BuiltinClass : Val
    {
        public static Dictionary<Type, BuiltinClass> BuiltinClasses = new Dictionary<Type, BuiltinClass>();

        public static BuiltinClass Bind(Type type)
        {
            BuiltinClass builtinClass;
            if (!BuiltinClasses.TryGetValue(type, out builtinClass))
            {
                builtinClass = BuiltinClasses[type] = new BuiltinClass();
                builtinClass.Init(type);
            }
            return builtinClass;
        }

        public string Name { get; private set; }

        public override List<Exprs.Expr> Body { get; protected set; }

        public override Scope Scope { get; protected set; }

        public Type BuiltinType { get; private set; }

        private BuiltinClass()
        {
        }

        private void Init(Type type)
        {
            BuiltinType = type;
            Attrs.BuiltinClass attr = type.GetCustomAttributes().OfType<Attrs.BuiltinClass>().FirstOrDefault();
            if (attr != null && !string.IsNullOrEmpty(attr.Name))
            {
                Name = attr.Name;
            }
            else
            {
                Name = type.Name;
            }
            Scope = new Scope(null);
            Body = new List<Exprs.Expr>();
            foreach (MethodInfo method in BuiltinType.GetMethods().Where(m => m.GetCustomAttributes().OfType<Attrs.BuiltinFunc>().Any()))
            {
                if (method.IsStatic)
                {
                    Scope.Add(method.Name, new BuiltinFunc(method, null));
                }
                else
                {
                    Body.Add(new Exprs.BuiltinFunc(method));
                }
            }
        }

        public override bool CanApply(params Val[] args)
        {
            return BuiltinType.GetConstructors().Any(ctor => BuiltinFunc.CanMethodApply(ctor, args));
        }

        public override Val Apply(params Val[] args)
        {
            List<ConstructorInfo> ctors = BuiltinType.GetConstructors().Where(ctor => BuiltinFunc.CanMethodApply(ctor, args)).ToList();
            if (ctors.Count > 1)
            {
                throw new Exception("ambiguous ctor for type " + Name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
            else if (ctors.Count < 1)
            {
                throw new Exception("unknown ctor for type " + Name + " with args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
            else
            {
                Val val = ctors.First().Invoke(args.ToArray()) as Val;
                if (val == null)
                {
                    throw new Exception("non-val builtinClass");
                }
                else
                {
                    return val;
                }
            }
        }

        public override string ToString()
        {
            return Name;
        }
    }
}