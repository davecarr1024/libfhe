using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace terp
{
  public class Parser
  {
    public class Result
    {
      public string Type { get; private set; }

      public string Value { get; private set; }

      public Result Parent { get; private set; }

      public List<Result> Children { get; private set; }

      public Result(string type, string value, params Result[] children)
      {
        Type = type;
        Value = value;
        Children = new List<Result>(children);
        Children.ForEach(child => child.Parent = this);
      }

      public bool Equals(Result result)
      {
        if (Type != result.Type)
        {
          return false;
        }
        else if (Value != result.Value)
        {
          return false;
        }
        else if (Children.Count != result.Children.Count)
        {
          return false;
        }
        else
        {
          for (int i = 0; i < Children.Count; ++i)
          {
            if (!Children[i].Equals(result.Children[i]))
            {
              return false;
            }
          }
          return true;
        }
      }

      public void AddChild(Result child)
      {
        if (child.Parent != this)
        {
          child.Parent = this;
          Children.Add(child);
        }
      }
    };

    public enum RuleType
    {
      Terminal,
      And,
      Or,
      ZeroOrMore,
      OneOrMore,
    };

    public class RuleDef
    {
      public RuleType Type { get; private set; }

      public string Name { get; private set; }

      public List<string> Children { get; private set; }

      public RuleDef(RuleType type, string name, params string[] children)
      {
        Type = type;
        Name = name;
        Children = new List<string>(children);
      }
    };

    public class Rule
    {
      public RuleType Type { get; private set; }

      public string Name { get; private set; }

      public List<Rule> Children { get; private set; }

      public Rule(RuleType type, string name)
      {
        Type = type;
        Name = name;
        Children = new List<Rule>();
      }

      public Result Parse(ref Queue<Lexer.Result> input)
      {
        switch (Type)
        {
          case RuleType.Terminal:
            if (!input.Any())
            {
              throw new Exception("terminal rule " + Name + " failed: out of input");
            }
            else
            {
              Lexer.Result token = input.Dequeue();
              if (token.Type != Name)
              {
                throw new Exception("terminal rule " + Name + " got mismatched token " + token.Type);
              }
              else
              {
                return new Result(Name, token.Value);
              }
            }
          case RuleType.And:
            {
              Result result = new Result(Name, null);
              foreach (Rule childRule in Children)
              {
                try
                {
                  result.AddChild(childRule.Parse(ref input));
                }
                catch (Exception ex)
                {
                  throw new Exception("and rule " + Name + " failed to add child " + childRule.Name + ": " + ex.Message);
                }
              }
              return result;
            }
          case RuleType.Or:
            {
              List<Exception> childExceptions = new List<Exception>();
              foreach (Rule childRule in Children)
              {
                Result result = new Result(Name, null);
                Queue<Lexer.Result> childInput = new Queue<Lexer.Result>(input);
                Result childResult;
                try
                {
                  childResult = childRule.Parse(ref childInput);
                }
                catch (Exception ex)
                {
                  childResult = null;
                  childExceptions.Add(ex);
                }
                if (childResult != null)
                {
                  result.AddChild(childResult);
                  input = new Queue<Lexer.Result>(childInput);
                  return result;
                }
              }
              throw new Exception("or rule " + Name + " failed to add any children:\r\n  " + string.Join("\r\n  ", childExceptions.ConvertAll(ex => ex.Message).ToArray()));
            }
          case RuleType.ZeroOrMore:
              if (Children.Count != 1)
              {
                throw new Exception("zero or more rule " + Name + " must have 1 child");
              }
              else
              {
              Result result = new Result(Name, null);
              bool added = true;
              while (added)
              {
                Queue<Lexer.Result> childInput = new Queue<Lexer.Result>(input);
                added = true;
                try
                {
                  result.AddChild(Children.First().Parse(ref childInput));
                }
                catch (Exception)
                {
                  added = false;
                }
                if (added)
                {
                  input = new Queue<Lexer.Result>(childInput);
                }
              }
              return result;
              }
          case RuleType.OneOrMore:
              if (Children.Count != 1)
              {
                throw new Exception("one or more rule " + Name + " must have 1 child");
              }
              else
              {
                Result result = new Result(Name, null);
                Result childResult;
                Queue<Lexer.Result> childInput = new Queue<Lexer.Result>(input);
                try
                {
                  childResult = Children.First().Parse(ref childInput);
                }
                catch (Exception ex)
                {
                  throw new Exception("one or more rule " + Name + " failed to parse first child: " + ex.Message);
                }
                while (childResult != null)
                {
                  input = new Queue<Lexer.Result>(childInput);
                  result.AddChild(childResult);
                  try
                  {
                    childResult = Children.First().Parse(ref childInput);
                  }
                  catch (Exception)
                  {
                    childResult = null;
                  }
                }
                return result;
              }
          default:
            throw new NotImplementedException();
        }
      }
    };

    public Lexer Lexer { get; private set; }

    public Rule RootRule { get; private set; }

    public void Connect(List<RuleDef> defs)
    {
      Dictionary<string, Rule> rules = new Dictionary<string, Rule>();
      foreach (RuleDef def in defs)
      {
        if (rules.ContainsKey(def.Name))
        {
          throw new Exception("duplicate rule " + def.Name);
        }
        rules[def.Name] = new Rule(def.Type, def.Name);
      }

      foreach (RuleDef def in defs)
      {
        Rule rule = rules[def.Name];
        if (RootRule == null)
        {
          RootRule = rule;
        }
        foreach (string child in def.Children)
        {
          if (!rules.ContainsKey(child))
          {
            if (Lexer.HasRule(child))
            {
              rules[child] = new Rule(RuleType.Terminal, child);
            }
            else
            {
              throw new Exception("rule " + rule.Name + " child " + child + " not found");
            }
          }
          rule.Children.Add(rules[child]);
        }
      }
    }

    public Parser(Lexer lexer, params RuleDef[] defsArray)
    {
      Lexer = lexer;
      Connect(new List<RuleDef>(defsArray));
    }

    public Result Parse(string input)
    {
      if (RootRule == null)
      {
        throw new Exception("unable to parse: no rules provided");
      }
      Queue<Lexer.Result> tokens = new Queue<Lexer.Result>(Lexer.Lex(input));
      Result result = RootRule.Parse(ref tokens);
      if (tokens.Any())
      {
        throw new Exception("parse had leftover tokens: " + string.Join(" ", tokens.Select(token => token.Value).ToArray()));
      }
      return result;
    }

    public List<Result> ParseMultiple(string input)
    {
      if (RootRule == null)
      {
        throw new Exception("unable to parse: no rules provided");
      }
      Queue<Lexer.Result> tokens = new Queue<Lexer.Result>(Lexer.Lex(input));
      List<Result> results = new List<Result>();
      while (tokens.Any())
      {
        results.Add(RootRule.Parse(ref tokens));
      }
      return results;
    }

    public Parser(string input)
    {
      /**
       * id = "[a-zA-Z]\w*"
       * equals = "\="
       * plus = "\+"
       * star = "\*"
       * lparen = "\("
       * rparen = "\)"
       * cr = "\r"
       * lf = "\n"
       * comment = "\#([^\r\n].)+"
       * string = "\"((\\.)|[^\\\\\"])*\""
       * whitespace *= "[ \t]+"
       * 
       * grammar = line*
       * newline = cr | lf | crlf | comment
       * crlf = cr lf
       * line = token | delimToken | rule | newline
       * comment = hash commentContent
       * token = id equals string newline
       * delimToken = id star equals string newline
       * rule = id equals ruleDecl newline
       * ruleDecl = ruleDeclAnd | ruleDeclOr | ruleDeclStar | ruleDeclPlus
       * ruleDeclAnd = id id ruleDeclAndTail
       * ruleDeclAndTail = id*
       * ruleDeclOr = id pipe id ruleDeclOrTail
       * ruleDeclOrTail = ruleDeclOrIter*
       * ruleDeclOrIter = pipe id
       * ruleDeclStar = id star
       * ruleDeclPlus = id plus
       */

      Lexer lexer = new Lexer(
        new Lexer.Rule("id", @"[a-zA-Z]\w*", false),
        new Lexer.Rule("string", "\".*\"", false),
        new Lexer.Rule("equals", @"\=", false),
        new Lexer.Rule("pipe", @"\|", false),
        new Lexer.Rule("star", @"\*", false),
        new Lexer.Rule("plus", @"\+", false),
        new Lexer.Rule("cr", "\r", false),
        new Lexer.Rule("lf", "\n", false),
        new Lexer.Rule("comment", @"\#.*\n", false),
        new Lexer.Rule("whitespace", @"\s+", true)
      );

      Parser grammarParser = new Parser(lexer,
        new RuleDef(RuleType.ZeroOrMore, "grammar", "line"),
        new RuleDef(RuleType.Or, "newline", "cr", "lf", "crlf", "comment"),
        new RuleDef(RuleType.And, "crlf", "cr", "lf"),
        new RuleDef(RuleType.Or, "line", "token", "delimToken", "rule", "newline"),
        new RuleDef(RuleType.And, "token", "id", "equals", "string", "newline"),
        new RuleDef(RuleType.And, "delimToken", "id", "star", "equals", "string", "newline"),
        new RuleDef(RuleType.And, "rule", "id", "equals", "ruleDecl", "newline"),
        new RuleDef(RuleType.Or, "ruleDecl", "ruleDeclAnd", "ruleDeclOr", "ruleDeclStar", "ruleDeclPlus"),
        new RuleDef(RuleType.And, "ruleDeclAnd", "id", "id", "ruleDeclAndTail"),
        new RuleDef(RuleType.ZeroOrMore, "ruleDeclAndTail", "id"),
        new RuleDef(RuleType.And, "ruleDeclOr", "id", "pipe", "id", "ruleDeclOrTail"),
        new RuleDef(RuleType.ZeroOrMore, "ruleDeclOrTail", "ruleDeclOrIter"),
        new RuleDef(RuleType.And, "ruleDeclOrIter", "pipe", "id"),
        new RuleDef(RuleType.And, "ruleDeclStar", "id", "star"),
        new RuleDef(RuleType.And, "ruleDeclPlus", "id", "plus")
      );

      Result grammar = grammarParser.Parse(input);

      List<RuleDef> rules = new List<RuleDef>();

      Lexer = new Lexer();
      foreach (Result line in grammar.Children)
      {
        if (line.Children[0].Type == "token")
        {
          Result token = line.Children[0];
          string name = token.Children[0].Value;
          string regex = token.Children[2].Value;
          regex = regex.Substring(1, regex.Length - 2);
          Lexer.Rules.Add(new Lexer.Rule(name, regex, false));
        }
        else if (line.Children[0].Type == "delimToken")
        {
          Result token = line.Children[0];
          string name = token.Children[0].Value;
          string regex = token.Children[3].Value;
          regex = regex.Substring(1, regex.Length - 2);
          Lexer.Rules.Add(new Lexer.Rule(name, regex, true));
        }
        else if (line.Children[0].Type == "rule")
        {
          Result ruleLine = line.Children[0];
          string name = ruleLine.Children[0].Value;
          Result ruleDecl = ruleLine.Children[2];
          rules.Add(ReadRule(name, ruleDecl));
        }
      }
      Connect(rules);
    }

    private RuleDef ReadRule(string name, Result ruleDecl)
    {
      Result rule = ruleDecl.Children[0];
      if (rule.Type == "ruleDeclAnd")
      {
        RuleDef def = new RuleDef(RuleType.And, name, rule.Children[0].Value, rule.Children[1].Value);
        Result tail = rule.Children[2];
        foreach (Result id in tail.Children)
        {
          def.Children.Add(id.Value);
        }
        return def;
      }
      else if (rule.Type == "ruleDeclOr")
      {
        RuleDef def = new RuleDef(RuleType.Or, name, rule.Children[0].Value, rule.Children[2].Value);
        Result tail = rule.Children[3];
        foreach (Result iter in tail.Children)
        {
          def.Children.Add(iter.Children[1].Value);
        }
        return def;
      }
      else if (rule.Type == "ruleDeclStar")
      {
        return new RuleDef(RuleType.ZeroOrMore, name, rule.Children[0].Value);
      }
      else if (rule.Type == "ruleDeclPlus")
      {
        return new RuleDef(RuleType.OneOrMore, name, rule.Children[0].Value);
      }
      else
      {
        throw new NotImplementedException("rule type " + ruleDecl.Children[0].Type);
      }
    }
  }
}
