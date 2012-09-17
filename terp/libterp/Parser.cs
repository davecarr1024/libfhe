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
      public string Type { get; private set; }

      public string Value { get; private set; }

      public List<Result> Children { get; private set; }

      public Result(string type, string value)
      {
        Type = type;
        Value = value;
        Children = new List<Result>();
      }

      public Result(string type, params Result[] children)
      {
        Type = type;
        Children = new List<Result>(children);
      }
    };

    internal class Context
    {
      private List<Lexer.Token> tokens;

      private int position = 0;

      private Stack<int> PositionStack = new Stack<int>();

      public Context(List<Lexer.Token> tokens)
      {
        this.tokens = tokens;
      }

      public Lexer.Token Next()
      {
        if (position < tokens.Count)
        {
          return tokens[position++];
        }
        else
        {
          return null;
        }
      }

      public void Push()
      {
        PositionStack.Push(position);
      }

      public void Pop()
      {
        position = PositionStack.Pop();
      }
    }

    public class Rule
    {
      public enum Type
      {
        Token,
        Or,
        And,
        OneOrMore,
      };

      public string Name { get; private set; }

      public Type type { get; private set; }

      public List<Rule> Children { get; private set; }

      public Rule(Type type, string name, params Rule[] children)
      {
        this.type = type;
        Name = name;
        Children = new List<Rule>(children);
      }

      internal Result Parse(Context context)
      {
        switch (type)
        {
          case Type.Token:
            {
              context.Push();
              Lexer.Token token = context.Next();
              if (token != null && token.Type == Name)
              {
                return new Result(Name, token.Value);
              }
              context.Pop();
              return null;
            }
          case Type.Or:
            {
              foreach (Rule rule in Children)
              {
                context.Push();
                Result result = rule.Parse(context);
                if (result != null)
                {
                  return result;
                }
                context.Pop();
              }
              return null;
            }
          case Type.And:
            {
              context.Push();
              Result result = new Result(Name);
              foreach (Rule rule in Children)
              {
                Result childResult = rule.Parse(context);
                if (childResult == null)
                {
                  context.Pop();
                  return null;
                }
                else
                {
                  result.Children.Add(childResult);
                }
              }
              return result;
            }
          case Type.OneOrMore:
            {
              context.Push();
              Result result = new Result(Name);
              Result childResult = Children.First().Parse(context);
              if (childResult == null)
              {
                context.Pop();
                return null;
              }
              else
              {
                while (childResult != null)
                {
                  context.Push();
                  result.Children.Add(childResult);
                  childResult = Children.First().Parse(context);
                  if (childResult == null)
                  {
                    context.Pop();
                  }
                }
                return result;
              }
            }
          default:
            return null;
        }
      }
    }

    public Rule Root { get; set; }

    public Lexer Lexer { get; set; }

    public Result Parse(string s)
    {
      return Root.Parse(new Context(Lexer.Lex(s)));
    }

    public Parser(Rule root, Lexer lexer)
    {
      Root = root;
      Lexer = lexer;
    }

    public Parser(string s)
    {
      Lexer lexer = new Lexer(
        new Lexer.Rule("enddecl", @";"),
        new Lexer.Rule("ruleop", @"=>"),
        new Lexer.Rule("tokenop", @"="),
        new Lexer.Rule("orop", @"\|"),
        new Lexer.Rule("andop", @"\+"),
        new Lexer.Rule("lparen", @"\("),
        new Lexer.Rule("rparen", @"\)"),
        new Lexer.Rule("oneormoreop", @"\*"),
        new Lexer.Rule("delimop", @"\(delim\)"),
        new Lexer.Rule("id", @"\w+"),
        new Lexer.Rule("regex", @"\S+"),
        new Lexer.Rule("ws", @"\s+", true)
        );

      Parser parser = new Parser(
        new Rule(Rule.Type.OneOrMore, "grammar",
          new Rule(Rule.Type.And, "decl",
            new Rule(Rule.Type.Or, "decl2",
              new Rule(Rule.Type.And, "rule",
                new Rule(Rule.Type.Token, "id"),
                new Rule(Rule.Type.Token, "ruleop"),
                new Rule(Rule.Type.Token, "id")
              ),
              new Rule(Rule.Type.And, "token",
                new Rule(Rule.Type.Token, "id"),
                new Rule(Rule.Type.Token, "tokenop"),
                new Rule(Rule.Type.Token, "id")
              )
            ),
            new Rule(Rule.Type.Token, "enddecl")
          )
        ),
        lexer
      );

      Result result = parser.Parse(s);
    }

  }
}
