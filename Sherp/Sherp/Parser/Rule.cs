using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Sherp.Parser
{
    public class Rule
    {
        public enum Types
        {
            Terminal,
            And,
            Or,
            OneOrMore,
            ZeroOrMore,
            ZeroOrOne,
        }

        public string Name { get; set; }

        public Types Type { get; private set; }

        public List<Rule> Children { get; private set; }

        public Rule(string name, Types type, params Rule[] children)
        {
            Name = name;
            Type = type;
            Children = children.ToList();
        }

        public Result Apply(List<Lexer.Result> tokens, ref int tokenPos)
        {
            switch (Type)
            {
                case Types.Terminal:
                    if (tokenPos < tokens.Count && tokens[tokenPos].Rule.Name == Name)
                    {
                        return new Result(this, tokens[tokenPos++].Value);
                    }
                    else
                    {
                        return null;
                    }
                case Types.And:
                    {
                        Result result = new Result(this);
                        foreach (Rule child in Children)
                        {
                            int childTokenPos = tokenPos;
                            Result childResult = child.Apply(tokens, ref childTokenPos);
                            if (childResult == null)
                            {
                                return null;
                            }
                            else
                            {
                                tokenPos = childTokenPos;
                                result.Children.Add(childResult);
                            }
                        }
                        return result;
                    }
                case Types.Or:
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
                case Types.OneOrMore:
                    if (Children.Count != 1)
                    {
                        throw new Exception("OneOrMore rules must have one child");
                    }
                    else
                    {
                        int childTokenPos = tokenPos;
                        Result childResult = Children[0].Apply(tokens, ref childTokenPos);
                        if (childResult == null)
                        {
                            return null;
                        }
                        else
                        {
                            tokenPos = childTokenPos;
                            Result result = new Result(this, childResult);
                            while (true)
                            {
                                childResult = Children[0].Apply(tokens, ref childTokenPos);
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
                case Types.ZeroOrMore:
                    if ( Children.Count != 1)
                    {
                        throw new Exception("ZeroOrMore rules must have one child");
                    }
                    else
                    {
                        Result result = new Result(this);
                        while (true)
                        {
                            int childTokenPos = tokenPos;
                            Result childResult = Children[0].Apply(tokens, ref childTokenPos);
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
                case Types.ZeroOrOne:
                    if (Children.Count != 1)
                    {
                        throw new Exception("ZeroOrOne rules must have one child");
                    }
                    else
                    {
                        int childTokenPos = tokenPos;
                        Result childResult = Children[0].Apply(tokens, ref childTokenPos);
                        if (childResult == null)
                        {
                            return null;
                        }
                        else
                        {
                            tokenPos = childTokenPos;
                            return new Result(this, childResult);
                        }
                    }
                default:
                    throw new NotImplementedException();
            }
        }

        public override string ToString()
        {
            return string.Format("<Parser.Rule Name=\"{0}\" Type=\"{1}\"/>", Name, Type);
        }
    }
}
