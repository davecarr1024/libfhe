using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace derp
{
    public class Parser
    {
        public class Result
        {
            public Rule Rule { get; set; }

            public List<Result> Children { get; set; }

            public string Value { get; set; }
        }

        public class Rule
        {
            public enum Types
            {
                And,
                Or,
                OneOrMore,
                ZeroOrMore,
                Terminal,
            }

            public string Name { get; set; }

            public Types Type { get; set; }

            public List<Rule> Children { get; set; }

            public Result Apply(List<Lexer.Result> tokens, ref int tokenPosition)
            {
                switch (Type)
                {
                    case Types.And:
                        {
                            List<Result> childResults = new List<Result>();
                            foreach (Rule child in Children)
                            {
                                Result childResult = child.Apply(tokens, ref tokenPosition);
                                if (childResult == null)
                                {
                                    return null;
                                }
                                else
                                {
                                    childResults.Add(childResult);
                                }
                            }
                            return new Result() { Rule = this, Children = childResults, Value = null };
                        }
                    case Types.Or:
                        {
                            foreach (Rule child in Children)
                            {
                                int pos = tokenPosition;
                                Result childResult = child.Apply(tokens, ref tokenPosition);
                                if (childResult == null)
                                {
                                    tokenPosition = pos;
                                }
                                else
                                {
                                    return new Result() { Rule = this, Children = new List<Result>() { childResult }, Value = null };
                                }
                            }
                            return null;
                        }
                    case Types.OneOrMore:
                        {
                            if (Children.Count != 1)
                            {
                                throw new Exception("OneOrMore rules must have one child");
                            }
                            Result childResult = Children[0].Apply(tokens, ref tokenPosition);
                            if (childResult == null)
                            {
                                return null;
                            }
                            else
                            {
                                List<Result> childResults = new List<Result>() { childResult };
                                while (true)
                                {
                                    int pos = tokenPosition;
                                    childResult = Children[0].Apply(tokens, ref tokenPosition);
                                    if (childResult == null)
                                    {
                                        tokenPosition = pos;
                                        break;
                                    }
                                    else
                                    {
                                        childResults.Add(childResult);
                                    }
                                }
                                return new Result() { Rule = this, Children = childResults, Value = null };
                            }
                        }
                    case Types.ZeroOrMore:
                        {
                            if (Children.Count != 1)
                            {
                                throw new Exception("ZeroOrMore rules must have one child");
                            }
                            List<Result> childResults = new List<Result>();
                            while (true)
                            {
                                int pos = tokenPosition;
                                Result childResult = Children[0].Apply(tokens, ref tokenPosition);
                                if (childResult == null)
                                {
                                    tokenPosition = pos;
                                    return new Result() { Rule = this, Children = childResults, Value = null };
                                }
                                else
                                {
                                    childResults.Add(childResult);
                                }
                            }
                        }
                    case Types.Terminal:
                        {
                            if (tokenPosition > tokens.Count)
                            {
                                throw new Exception("parse overflow");
                            }
                            else if (tokens[tokenPosition].Rule.Name == Name)
                            {
                                return new Result() { Rule = this, Children = new List<Result>(), Value = tokens[tokenPosition++].Value };
                            }
                            else
                            {
                                return null;
                            }
                        }
                    default:
                        throw new NotImplementedException();
                }
            }
        }

        public Result Parse(string input)
        {
            List<Lexer.Result> tokens = Lexer.Lex(input);
            int tokenPosition = 0;
            Result result = Root.Apply(tokens, ref tokenPosition);
            if (tokenPosition != tokens.Count)
            {
                throw new Exception("parse underflow");
            }
            else
            {
                return result;
            }
        }

        public Lexer Lexer { get; set; }

        public Rule Root { get; set; }
    }
}
