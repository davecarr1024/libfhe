using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Vals
{
    public class Func : ApplyVal
    {
        public Val Type { get { return Class.Bind(GetType()); } }

        public bool IsReturn { get; set; }

        public string Name { get; private set; }

        public Val ReturnType { get; private set; }

        public List<Param> Params { get; private set; }

        public List<Exprs.Expr> Body { get; private set; }

        public List<List<Param>> ParamsList { get { return new List<List<Param>>() { Params }; } }

        public Func(string name, Val returnType, List<Param> paramList, List<Exprs.Expr> body)
        {
            IsReturn = false;
            Name = name;
            ReturnType = returnType;
            Params = paramList;
            Body = body;
        }

        public Val Apply(List<Exprs.Expr> args, Scope scope)
        {
            if (args.Count != Params.Count)
            {
                throw new Exception("arg count mismatch");
            }
            else
            {
                Scope funcScope = new Scope(scope);
                for (int i = 0; i < args.Count; ++i)
                {
                    Val argVal = args[i].Eval(funcScope);
                    if (argVal.Type != Params[i].Type)
                    {
                        throw new Exception("unable to cast arg " + Params[i].Name + " from " + argVal.Type + " to " + Params[i].Type);
                    }
                }
                foreach (Exprs.Expr expr in Body)
                {
                    Val val = expr.Eval(funcScope);
                    if (val.IsReturn)
                    {
                        if (ReturnType == null)
                        {
                            if (val is NoneType)
                            {
                                return val;
                            }
                            else
                            {
                                throw new Exception("func " + Name + " has void return type but tried to return val");
                            }
                        }
                        else if (val.Type != ReturnType)
                        {
                            throw new Exception("func " + Name + " unable to cast return val from " + val.Type + " to " + ReturnType);
                        }
                        else
                        {
                            return val;
                        }
                    }
                }
                return new NoneType();
            }
        }

        public bool ToBool()
        {
            return true;
        }
    }
}
