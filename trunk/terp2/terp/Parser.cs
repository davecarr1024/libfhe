using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace terp
{
  public class Parser
  {
    public enum RuleType
    {
      Terminal,
      Or,
      And,
      OneOrMore,
      ZeroOrMore,
      ZeroOrOne,
    }

    public class Rule
    {
      public RuleType Type { get; set; }

      public string Name { get; set; }

      public List<Rule> Children { get; set; }

      public Result Parse(ref Queue<Lexer.Result> input)
      {
        switch (Type)
        {
          case RuleType.Terminal:
            return ParseTerminal(input);
          case RuleType.Or:
            return ParseOr(ref input);
          case RuleType.And:
            return ParseAnd(ref input);
          case RuleType.ZeroOrMore:
            return ParseZeroOrMore(ref input);
          case RuleType.ZeroOrOne:
            return ParseZeroOrOne(ref input);
          case RuleType.OneOrMore:
            return ParseOneOrMore(ref input);
          default:
            throw new NotImplementedException();
        }
      }

      private Result ParseOneOrMore(ref Queue<Lexer.Result> input)
      {
        Result result = new Result() { Type = Name };
        try
        {
          result.Children.Add(Children.First().Parse(ref input));
        }
        catch (Exception ex)
        {
          throw new Exception("one or more rule " + Name + " failed: failed to parse first child: " + ex);
        }
        bool parsed = true;
        while (parsed)
        {
          try
          {
            result.Children.Add(Children.First().Parse(ref input));
          }
          catch (Exception)
          {
            parsed = false;
          }
        }
        return result;
      }

      private Result ParseZeroOrOne(ref Queue<Lexer.Result> input)
      {
        Result result = new Result() { Type = Name };
        try
        {
          result.Children.Add(Children.First().Parse(ref input));
        }
        catch (Exception)
        {
        }
        return result;
      }

      private Result ParseZeroOrMore(ref Queue<Lexer.Result> input)
      {
        Result result = new Result() { Type = Name };
        bool parsed = true;
        while (parsed)
        {
          try
          {
            result.Children.Add(Children.First().Parse(ref input));
          }
          catch (Exception)
          {
            parsed = false;
          }
        }
        return result;
      }

      private Result ParseAnd(ref Queue<Lexer.Result> input)
      {
        Result result = new Result() { Type = Name };
        foreach (Rule childRule in Children)
        {
          try
          {
            result.Children.Add(childRule.Parse(ref input));
          }
          catch (Exception ex)
          {
            throw new Exception("and rule " + Name + " failed: failed to apply child " + childRule + ": " + ex);
          }
        }
        return result;
      }

      private Result ParseOr(ref Queue<Lexer.Result> input)
      {
        List<Exception> childExceptions = new List<Exception>();
        foreach (Rule childRule in Children)
        {
          Queue<Lexer.Result> childInput = new Queue<Lexer.Result>(input);
          Result childResult;
          try
          {
            childResult = childRule.Parse(ref childInput);
          }
          catch (Exception ex)
          {
            childExceptions.Add(ex);
            childResult = null;
          }
          if (childResult != null)
          {
            input = new Queue<Lexer.Result>(childInput);
            return new Result() { Type = Name, Children = { childResult } };
          }
        }
        throw new Exception("or rule " + Name + " failed: failed to apply any children " + childExceptions);
      }

      private Result ParseTerminal(Queue<Lexer.Result> input)
      {
        Lexer.Result token = input.Dequeue();
        if (token.Type == Name)
        {
          return new Result() { Type = Name, Value = token.Value };
        }
        else
        {
          throw new Exception("terminal rule " + Name + " failed: mismatched token " + token.Type);
        }
      }

      public Rule()
      {
        Children = new List<Rule>();
      }
    }

    public class Result
    {
      public string Type { get; set; }

      public string Value { get; set; }

      public List<Result> Children { get; set; }

      public Result()
      {
        Children = new List<Result>();
      }
    }

    public Lexer Lexer { get; set; }

    public Rule RootRule { get; set; }

    public Result Parse(string input)
    {
      Queue<Lexer.Result> tokens = new Queue<Lexer.Result>(Lexer.Lex(input));
      Result result = null;
      while (tokens.Any())
      {
        result = RootRule.Parse(ref tokens);
      }
      return result;
    }
  }
}
