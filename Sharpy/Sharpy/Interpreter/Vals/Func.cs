using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Interpreter.Vals
{
    public class Func : Val
    {
        public Val Type { get { return BuiltinClass.Bind(GetType()); } }

        public string Name { get; private set; }

        public List<Param> Params { get; private set; }

        public Val ReturnType { get; private set; }

        public List<Exprs.Expr> Body { get; private set; }

        public Scope Scope { get; private set; }

        public bool IsReturn { get; set; }

        public Func(string name, List<Param> paramList, Val returnType, List<Exprs.Expr> body, Scope scope)
        {
            Name = name;
            Params = paramList;
            ReturnType = returnType;
            Body = body;
            Scope = scope;
            IsReturn = false;
        }

        public bool CanApply(params Val[] argTypes)
        {
            return
                Params.Count == argTypes.Length &&
                Enumerable.Range(0, Params.Count).All(i => Interpreter.CanConvert(argTypes[i], Params[i].Type));
        }

        public Val Apply(params Val[] args)
        {
            if (CanApply(args.Select(arg => arg.Type).ToArray()))
            {
                Scope scope = new Scope(Scope);
                for (int i = 0; i < args.Length; ++i)
                {
                    scope.Add(Params[i].Type, Params[i].Name, args[i]);
                }
                Val ret = Interpreter.Eval(scope, Body.ToArray());
                if (Interpreter.CanConvert(ret.Type, ReturnType))
                {
                    return Interpreter.Convert(ret, ReturnType);
                }
                else
                {
                    throw new Exception("func " + Name + " unable to convert ret val " + ret + " of type " + ret.Type + " to ret type " + ReturnType);
                }
            }
            else
            {
                throw new Exception("func " + Name + " can't apply args [" + string.Join(", ", args.Select(arg => arg.ToString()).ToArray()) + "]");
            }
        }

        public override string ToString()
        {
            if (Params.Any())
            {
                return string.Format("{0} {1}( {2} )", ReturnType, Name, string.Join(", ", Params.Select(p => p.ToString()).ToArray()));
            }
            else
            {
                return string.Format("{0} {1}()", ReturnType, Name);
            }
        }
    }
}
