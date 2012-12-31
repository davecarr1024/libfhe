from copy import copy
from Lexer import Lexer

class Parser:

  class Result:
    def __init__( self, name, value, children ):
      self.name = name
      self.value = value
      self.children = children
    
    def __eq__( self, result ):
      return self.name == result.name and self.value == result.value and self.children == result.children
      
    def __repr__( self ):
      return 'Parser.Result( %s, %s, %s )' % ( self.name, self.value, self.children )
      
    def tree( self, tabs = 0 ):
      return '%s%s %s\n' % ( '  ' * tabs, self.name, self.value ) + \
        ''.join( [ child.tree( tabs + 1 ) for child in self.children ]  )

  class Rule:
    AND = 'and'
    OR = 'or'
    LITERAL = 'literal'
    ONE_OR_MORE = 'one_or_more'
    ZERO_OR_MORE = 'zero_or_more'
  
    def __init__( self, name, type, children ):
      self.name = name
      self.type = type
      self.children = children
      
    def __repr__( self ):
      return 'Parser.Rule( %s, %s, %s )' % ( self.name, self.type, self.children )
      
    def tree( self, tabs = 0 ):
      return '%s%s %s\n' % ( '  ' * tabs, self.name, self.type ) + \
        ''.join( [ child.tree( tabs + 1 ) for child in self.children ] )
      
    def parse( self, tokens ):
    
      if self.type == self.AND:
        childResults = []
        for child in self.children:
          childResultAndTokens = child.parse( tokens )
          if not childResultAndTokens:
            return
          childResult, tokens = childResultAndTokens
          childResults.append( childResult )
        return Parser.Result( self.name, None, childResults ), tokens
        
      elif self.type == self.OR:
        bestResult = None
        
        for child in self.children:
          childResultAndTokens = child.parse( copy( tokens ) )
          if childResultAndTokens:
            childResult, childTokens = childResultAndTokens
            if not bestResult or len( childTokens ) < len( bestTokens ):
              bestResult = childResult
              bestTokens = childTokens
            
        if bestResult:
          return Parser.Result( self.name, None, [ bestResult ] ), bestTokens
            
      elif self.type == self.LITERAL:
        if tokens and tokens[0].name == self.name:
          return Parser.Result( self.name, tokens[0].value, [] ), tokens[1:]
          
      elif self.type == self.ONE_OR_MORE:
        assert len( self.children ) == 1
        childResultAndTokens = self.children[0].parse( tokens )
        if childResultAndTokens:
          childResult, tokens = childResultAndTokens
          childResults = [ childResult ]
          while True:
            childResultAndTokens = self.children[0].parse( tokens )
            if not childResultAndTokens:
              return Parser.Result( self.name, None, childResults ), tokens
            childResult, tokens = childResultAndTokens
            childResults.append( childResult )
          
      elif self.type == self.ZERO_OR_MORE:
        assert len( self.children ) == 1
        childResults = []
        while True:
          childResultAndTokens = self.children[0].parse( tokens )
          if not childResultAndTokens:
            return Parser.Result( self.name, None, childResults ), tokens
          childResult, tokens = childResultAndTokens
          childResults.append( childResult )
        
      else:
        raise RuntimeError
      
  def __init__( self, lexer, root ):
    self.lexer = lexer
    self.root = root
    
  def parse( self, input ):
    tokens = self.lexer.lex( input )
    resultAndTokens = self.root.parse( tokens )
    assert resultAndTokens, 'failed to parse %s' % tokens
    result, tokens = resultAndTokens
    assert not tokens, 'leftover tokens %s\n%s' % ( ' '.join( [ token.name for token in tokens ] ), result.tree() )
    return result
    
  @staticmethod
  def load( input ):
    lexer = Lexer( [ Lexer.Rule( 'str', '\"[\S \t]*\"', False ),
                     Lexer.Rule( 'id', '\w+', False ),
                     Lexer.Rule( 'equals', '=', False ),
                     Lexer.Rule( 'star', '\*', False ),
                     Lexer.Rule( 'plus', '\+', False ),
                     Lexer.Rule( 'pipe', '\|', False ),
                     Lexer.Rule( 'tilde', '\~', False ),
                     Lexer.Rule( 'nl', '[\n\r]', False ),
                     Lexer.Rule( 'ws', '[ \t]+', True ),
                     Lexer.Rule( 'comment', '#[\S \t]*[\n\r]', True ),
                  ] )
                  
    parser = Parser( lexer,
                    Parser.Rule( 'rules', Parser.Rule.ONE_OR_MORE, [
                      Parser.Rule( 'line', Parser.Rule.OR, [
                        Parser.Rule( 'nl', Parser.Rule.LITERAL, [] ),
                        Parser.Rule( 'lexerRule', Parser.Rule.AND, [
                          Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'equals', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'str', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'nl', Parser.Rule.LITERAL, [] ),
                        ] ),
                        Parser.Rule( 'lexerExcludeRule', Parser.Rule.AND, [
                          Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'equals', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'tilde', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'str', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'nl', Parser.Rule.LITERAL, [] ),
                        ] ),
                        Parser.Rule( 'parserRule', Parser.Rule.AND, [
                          Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'equals', Parser.Rule.LITERAL, [] ),
                          Parser.Rule( 'parserRuleDef', Parser.Rule.OR, [
                            Parser.Rule( 'parserRuleAnd', Parser.Rule.AND, [
                              Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                              Parser.Rule( 'parserRuleAndTail', Parser.Rule.ONE_OR_MORE, [
                                Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                              ] ),
                            ] ),
                            Parser.Rule( 'parserRuleOneOrMore', Parser.Rule.AND, [
                              Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                              Parser.Rule( 'plus', Parser.Rule.LITERAL, [] ),
                            ] ),
                            Parser.Rule( 'parserRuleZeroOrMore', Parser.Rule.AND, [
                              Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                              Parser.Rule( 'star', Parser.Rule.LITERAL, [] ),
                            ] ),
                            Parser.Rule( 'parserRuleOr', Parser.Rule.AND, [
                              Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                              Parser.Rule( 'parserRuleOrTail', Parser.Rule.ONE_OR_MORE, [
                                Parser.Rule( 'parserRuleOrTailDef', Parser.Rule.AND, [
                                  Parser.Rule( 'pipe', Parser.Rule.LITERAL, [] ),
                                  Parser.Rule( 'id', Parser.Rule.LITERAL, [] ),
                                ] ),
                              ] ),
                            ] ),
                          ] ),
                          Parser.Rule( 'nl', Parser.Rule.LITERAL, [] ),
                        ] ),
                      ] ),
                    ] )
                    )
                    
    result = parser.parse( input )
    
    assert result
    
    lexerRules = {}
    parserRules = {}
    for rule in result.children:
      ruleDef = rule.children[0]
      if ruleDef.name == 'lexerRule':
        name = ruleDef.children[0].value
        lexerRules[ name ] = Lexer.Rule( name,
                                         ruleDef.children[2].value[1:-1],
                                         False )
      elif ruleDef.name == 'lexerExcludeRule':
        name = ruleDef.children[0].value
        lexerRules[ name ] = Lexer.Rule( name,
                                         ruleDef.children[3].value[1:-1],
                                         True )
      elif ruleDef.name == 'parserRule':
        name = ruleDef.children[0].value
        parserRuleDef = ruleDef.children[2].children[0]
        if parserRuleDef.name == 'parserRuleAnd':
          ids = [ parserRuleDef.children[0].value ]
          for tail in parserRuleDef.children[1].children:
            ids.append( tail.value )
          parserRules[ name ] = Parser.Rule( name, Parser.Rule.AND, ids )
        elif parserRuleDef.name == 'parserRuleOneOrMore':
          id = parserRuleDef.children[0].value
          parserRules[ name ] = Parser.Rule( name, Parser.Rule.ONE_OR_MORE, [ id ] )
        elif parserRuleDef.name == 'parserRuleZeroOrMore':
          id = parserRuleDef.children[0].value
          parserRules[ name ] = Parser.Rule( name, Parser.Rule.ZERO_OR_MORE, [ id ] )
        elif parserRuleDef.name == 'parserRuleOr':
          ids = [ parserRuleDef.children[0].value ]
          for tail in parserRuleDef.children[1].children:
            ids.append( tail.children[1].value )
          parserRules[ name ] = Parser.Rule( name, Parser.Rule.OR, ids )
                                       
    for rule in parserRules.values():
      newChildren = []
      for child in rule.children:
        if child not in parserRules and child in lexerRules:
          parserRules[child] = Parser.Rule( child, Parser.Rule.LITERAL, [] )
        assert child in parserRules, 'rule %s references unknown rule %s' % ( rule.name, child )
        newChildren.append( parserRules[child] )
      rule.children = newChildren
      
    roots = filter( lambda rule: all( map( lambda r: r == rule or rule not in r.children, parserRules.values() ) ), parserRules.values() )
    assert roots, 'no roots'
    assert len( roots ) == 1, 'multiple roots %s' % roots
    
    return Parser( Lexer( lexerRules.values() ), roots[0] )

if __name__ == '__main__':
  parser = Parser.load( '''
    #this is my comment
    
    lparen = "\("
    rparen = "\)"
    int = "-?\d+"
    float = "-?\d+\.\d+"
    str = "\"[\S \t]*\""
    id = "\w+"
    ws = ~"\s+"
    
    exprs = expr+
    expr = lparen exprContents rparen
    exprContents = exprContent*
    exprContent = id | int | float | str | expr
  ''' )
  assert parser
  
  result = parser.parse( '( foo -2 10.2 "oh hello" ( bar ) )' )
  
  assert result == \
    Parser.Result( 'exprs', None, [
      Parser.Result( 'expr', None, [
        Parser.Result( 'lparen', '(', [] ),
        Parser.Result( 'exprContents', None, [
          Parser.Result( 'exprContent', None, [
            Parser.Result( 'id', 'foo', [] ),
          ] ),
          Parser.Result( 'exprContent', None, [
            Parser.Result( 'int', '-2', [] ),
          ] ),
          Parser.Result( 'exprContent', None, [
            Parser.Result( 'float', '10.2', [] ),
          ] ),
          Parser.Result( 'exprContent', None, [
            Parser.Result( 'str', '"oh hello"', [] ),
          ] ),
          Parser.Result( 'exprContent', None, [
            Parser.Result( 'expr', None, [
              Parser.Result( 'lparen', '(', [] ),
              Parser.Result( 'exprContents', None, [
                Parser.Result( 'exprContent', None, [
                  Parser.Result( 'id', 'bar', [] ),
                ] ),
              ] ),
              Parser.Result( 'rparen', ')', [] ),
            ] ),
          ] ),
        ] ),
        Parser.Result( 'rparen', ')', [] ),
      ] ),
    ] )
