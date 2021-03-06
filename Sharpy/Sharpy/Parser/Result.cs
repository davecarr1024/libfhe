﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sharpy.Parser
{
    public class Result
    {
        public Rule Rule { get; private set; }

        public string Value { get; private set; }

        public List<Result> Children { get; private set; }

        public string Type { get { return Rule.Name; } }

        public Result(Rule rule, string value)
        {
            Rule = rule;
            Value = value;
            Children = new List<Result>();
        }

        public Result(Rule rule, params Result[] children)
        {
            Rule = rule;
            Value = null;
            Children = children.ToList();
        }

        public override string ToString()
        {
            return string.Format("Parser.Result({0},{1})", Rule.Name, Value);
        }

        public Result this[int i] { get { return Children[i]; } }
    }
}
