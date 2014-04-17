using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Interpreter.Exprs
{
    public class Func : Expr
    {
        public string Name { get; private set; }

        public string ReturnType { get; private set; }

        public List<Param> Params { get; private set; }

        public List<Expr> Body { get; private set; }

        public Func(string name, string returnType, List<Param> paramList, List<Expr> body)
        {
            Name = name;
            ReturnType = returnType;
            Params = paramList;
            Body = body;
        }

        public Vals.Val Eval(Scope scope)
        {
            Vals.Val returnType;
            if (ReturnType == "void")
            {
                returnType = null;
            }
            else if (!scope.TryGetValue(ReturnType, out returnType))
            {
                throw new Exception("func " + Name + " has unknown return type " + ReturnType);
            }

            List<Vals.Param> paramList = new List<Vals.Param>();
            foreach (Param param in Params)
            {
                Vals.Val paramType;
                if (!scope.TryGetValue(param.Type, out paramType))
                {
                    throw new Exception("func " + Name + " param " + param.Name + " has unknown type " + param.Type);
                }
                else
                {
                    paramList.Add(new Vals.Param(paramType, param.Name));
                }
            }
            return new Vals.Func(Name, returnType, paramList, Body);
        }
    }
}
