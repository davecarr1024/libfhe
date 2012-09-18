using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.ObjectModel;
using System.Text.RegularExpressions;

namespace libterp
{
    public class Parser
    {
        public class Result
        {
            public string Type;

            public string Value;

            public List<Result> Children { get; set; }

            public 
        };

        public class Context
        {
            public List<Lexer.Result> Tokens { get; set; }

            public int Position { get; set; }

            public Stack<Rule> Rules { get; set; }

            public Stack<Result> Results { get; set; }

            public Context()
            {
                Tokens = new List<Lexer.Result>();
                Position = 0;
                Rules = new Stack<Rule>();
                Results = new Stack<Result>();
            }

            private Context(Context context)
            {
                Tokens = new List<Lexer.Result>(context.Tokens);
                Position = context.Position;
                Rules = new Stack<Rule>(Rules);
                Results = new Stack<Result>(Results);
            }

            public List<Context> Expand()
            {
                List<Context> contexts = new List<Context>();
                if (Rules.Any())
                {
                    Rule rule = Rules.Peek();
                    switch (rule.type)
                    {
                        case Rule.Type.Terminal:
                            if (Position < Tokens.Count && Tokens[Position].Type == rule.Name)
                            {
                                Context context = new Context(this);
                                context.Position++;
                                context.Results.Push(new Result()
                                {
                                    Type = rule.Name,
                                    Value = Tokens[Position].Value,
                                });
                                context.Rules.Pop();
                                contexts.Add(context);
                            }
                            break;
                        case Rule.Type.Or:
                            foreach (Rule childRule in rule.Children)
                            {
                                Context context = new Context(this);
                                context.Results.Push(new Result() { Type = rule.Name, });
                                context.Rules.Pop();
                                context.Rules.Push(childRule);
                                contexts.Add(context);
                            }
                            break;
                        case Rule.Type.And:
                            {
                                Context context = new Context(this);
                                context.Results.Push(new Result() { Type = rule.Name });
                                context.Rules.Pop();
                                foreach (Rule childRule in rule.Children)
                                {
                                    context.Rules.Push(childRule);
                                }
                                contexts.Add(context);
                            }
                            break;
                    }
                }
                return contexts;
            }
        };

        public class Rule
        {
            public enum Type
            {
                Terminal,
                Or,
                And,
            };

            public Type type { get; set; }

            public string Name { get; set; }

            public List<Rule> Children { get; set; }

            public List<string> ChildNames { get; set; }

            public Rule()
            {
                Children = new List<Rule>();
                ChildNames = new List<string>();
            }
        };

        public Dictionary<string, Rule> Rules { get; set; }

        public Rule Root { get; set; }

        public Lexer Lexer { get; set; }

        public void ConnectRules()
        {
            Rules.Values
                .ToList().ForEach(rule => rule.ChildNames
                    .Where(name => rule.Children.All(child => child.Name != name))
                    .ToList().ForEach(name => rule.Children.Add(Rules[name])));
        }

        public Result Parse(string str)
        {
            ConnectRules();
            Queue<Context> contexts = new Queue<Context>();
            Context context = new Context() { Tokens = Lexer.Lex(str) };
            context.Rules.Push(Root);

            while (contexts.Any())
            {
                if (!context.Rules.Any() && context.Position == context.Tokens.Count)
                {
                    return context.Results.First();
                }
                else
                {
                    context.Expand().ForEach(c => contexts.Enqueue(c));
                }
            }

            return null;
        }
    }
}
