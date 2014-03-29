using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp3
{
    public class Parser
    {
        public class Rule
        {
            public enum Types
            {
                AND,
                OR,
                ONEORMORE,
                ZEROORMORE,
                TERMINAL,
            }

            public Types Type { get; set; }

            public string Name { get; set; }

            public List<Rule> Children { get; set; }

            public Rule(Types type, string name, params Rule[] children)
            {
                Type = type;
                Name = name;
                Children = children.ToList();
            }

            public override string ToString()
            {
                return string.Format("<Parser.Rule Type=\"{0}\" Name=\"{1}\">", Type, Name);
            }

            public Result Apply(List<Lexer.Result> tokens, ref int tokenPos)
            {
                switch (Type)
                {
                    case Types.AND:
                        {
                            Result result = new Result(this);
                            foreach (Rule child in Children)
                            {
                                Result childResult = child.Apply(tokens, ref tokenPos);
                                if (childResult == null)
                                {
                                    return null;
                                }
                                else
                                {
                                    result.Children.Add(childResult);
                                }
                            }
                            return result;
                        }
                    case Types.OR:
                        foreach (Rule child in Children)
                        {
                            int childTokenPos = tokenPos;
                            Result childResult = child.Apply(tokens, ref childTokenPos);
                            if (childResult != null)
                            {
                                tokenPos = childTokenPos;
                                return new Result(this, childResult);
                            }
                        }
                        return null;
                    case Types.ONEORMORE:
                        if (Children.Count != 1)
                        {
                            throw new Exception("one or more rules must have one child");
                        }
                        else
                        {
                            Result childResult = Children.First().Apply(tokens, ref tokenPos);
                            if (childResult == null)
                            {
                                return null;
                            }
                            else
                            {
                                Result result = new Result(this, childResult);
                                while (true)
                                {
                                    int childTokenPos = tokenPos;
                                    childResult = Children.First().Apply(tokens, ref childTokenPos);
                                    if (childResult == null)
                                    {
                                        return result;
                                    }
                                    else
                                    {
                                        tokenPos = childTokenPos;
                                        result.Children.Add(childResult);
                                    }
                                }
                            }
                        }
                    case Types.ZEROORMORE:
                        if (Children.Count != 1)
                        {
                            throw new Exception("zero or more rules must have one child");
                        }
                        else
                        {
                            Result result = new Result(this);
                            while (true)
                            {
                                int childTokenPos = tokenPos;
                                Result childResult = Children.First().Apply(tokens, ref childTokenPos);
                                if (childResult == null)
                                {
                                    return result;
                                }
                                else
                                {
                                    tokenPos = childTokenPos;
                                    result.Children.Add(childResult);
                                }
                            }
                        }
                    case Types.TERMINAL:
                        if (tokenPos < tokens.Count && tokens[tokenPos].Rule.Name == Name)
                        {
                            Result result = new Result(this, tokens[tokenPos].Value);
                            tokenPos++;
                            return result;
                        }
                        else
                        {
                            return null;
                        }
                    default:
                        throw new Exception("unknown Parser.Rule type " + Type);
                }
            }
        }

        public class Result
        {
            public Rule Rule { get; set; }

            public string Value { get; set; }

            public List<Result> Children { get; set; }

            public Result(Rule rule, string value)
            {
                Rule = rule;
                Value = value;
                Children = null;
            }

            public Result(Rule rule, params Result[] children)
            {
                Rule = rule;
                Value = null;
                Children = children.ToList();
            }

            public override string ToString()
            {
                return string.Format("<Parser.Result Rule=\"{0}\" Value=\"{1}\">", Rule.Name, Value);
            }
        }

        public Lexer Lexer { get; set; }

        public Rule Root { get; set; }

        public Parser(Lexer lexer, Rule root)
        {
            Lexer = lexer;
            Root = root;
        }

        public Result Parse(string input)
        {
            List<Lexer.Result> tokens = Lexer.Lex(input);
            int tokenPos = 0;
            Result result = Root.Apply(tokens, ref tokenPos);
            if (tokenPos != tokens.Count)
            {
                throw new Exception("parse underflow");
            }
            else
            {
                return result;
            }
        }
    }
}
