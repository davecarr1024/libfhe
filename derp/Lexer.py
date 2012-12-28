import re

class Lexer:

  class Token:
    def __init__( self, name, value ):
      self.name = name
      self.value = value
      
    def __eq__( self, token ):
      return self.name == token.name and self.value == token.value
      
    def __repr__( self ):
      return 'Token( %s, \'%s\' )' % ( self.name, self.value )
      
  class Rule:
    def __init__( self, name, pattern, exclude ):
      self.name = name
      self.pattern = pattern
      self.exclude = exclude
      
    def __repr__( self ):
      return 'Rule( %s, %s, %s )' % ( self.name, self.pattern, self.exclude )
      
    def match( self, input ):
      result = re.match( self.pattern, input )
      if result:
        return Lexer.Token( self.name, result.group() )

  def __init__( self, rules ):
    self.rules = rules
    
  def lex( self, input ):
    pos = 0
    toks = []
    while pos < len( input ):
      rawResults = [ ( rule, rule.match( input[pos:] ) ) for rule in self.rules ]
      results = filter( lambda result: result[1], rawResults )
      assert results, 'no results in %s for %s' % ( rawResults, input[pos:] )
      bestResult = max( results, key = lambda result: len( result[1].value ) )
      if not bestResult[0].exclude:
        toks.append( bestResult[1] )
      pos += len( bestResult[1].value )
    return toks

if __name__ == '__main__':
  lexer = Lexer( [ Lexer.Rule( 'lparen', '\(', False ),
                   Lexer.Rule( 'rparen', '\)', False ),
                   Lexer.Rule( 'int', '-?\d+', False ), 
                   Lexer.Rule( 'float', '-?\d+\.\d+', False ),
                   Lexer.Rule( 'id', '\w+', False ),
                   Lexer.Rule( 'ws', '\s+', True ),
                 ] )
  results = lexer.lex( '( foo 1 -3.14 )' )
  assert results == [ Lexer.Token( 'lparen', '(' ),
                      Lexer.Token( 'id', 'foo' ), 
                      Lexer.Token( 'int', '1' ), 
                      Lexer.Token( 'float', '-3.14' ),
                      Lexer.Token( 'rparen', ')' ),
                    ]
