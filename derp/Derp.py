from Parser import Parser
from copy import copy
import operator

class Derp:
  class Value:
    NULL = 'null'
    BOOL = 'bool'
    INT = 'int'
    FLOAT = 'float'
    STR = 'str'
    FUNCTION = 'function'
  
    def __init__( self, type, value ):
      self.type = type
      self.value = value
    
    def __repr__( self ):
      return 'Derp.Value( %s, %s )' % ( self.type, self.value )
      
    def __eq__( self, value ):
      return value and self.type == value.type and self.value == value.value
      
    @staticmethod
    def fromPy( val ):
      valType = type( val ).__name__.upper()
      assert hasattr( Derp.Value, valType ), valType
      return Derp.Value( getattr( Derp.Value, valType ), val )

  def __init__( self ):
    self.parser = Parser.load( '''
      lparen = "\("
      rparen = "\)"
      lbrace = "\{"
      rbrace = "\}"
      comma = "\,"
      semicolon = "\;"
      equals = "="
      operator = "\+|\-|\*|\/|=="
      int = "-?\d+"
      float = "-?\d+\.\d+"
      str = "\"[\S \t^\"]*?\""
      id = "\w+"
      ws = ~"\s+"
      
      program = line*
      line = functionDef | statementLine
      statementLine = statement semicolon
      statement = expr | assignment
      assignment = id equals expr
      nonOperationExpr = id | int | float | str | functionCall
      expr = nonOperationExpr | operationExpr
      functionCall = id lparen exprList rparen
      functionDef = id lparen idList rparen lbrace functionBody rbrace
      functionBody = line*
      operationExpr = nonOperationExpr operator nonOperationExpr
      idList = id idListTail
      idListTail = idListTailDef*
      idListTailDef = comma id
      exprList = expr exprListTail
      exprListTail = exprListTailDef*
      exprListTailDef = comma expr
    ''' )
    
  def _eval( self, result, scope ):
    if result.name == 'program':
      return [ self._eval( child, scope ) for child in result.children ]
      
    elif result.name == 'functionBody':
      return [ self._eval( child, scope ) for child in result.children ][-1]
      
    elif result.name in [ 'line', 'statementLine', 'statement', 'expr', 'nonOperationExpr' ]:
      return self._eval( result.children[0], scope )
      
    elif result.name == 'int':
      return Derp.Value( Derp.Value.INT, int( result.value ) )
      
    elif result.name == 'float':
      return Derp.Value( Derp.Value.FLOAT, float( result.value ) )
      
    elif result.name == 'str':
      return Derp.Value( Derp.Value.STR, result.value[1:-1] )
      
    elif result.name == 'id':
      assert result.value in scope, 'unknown var %s' % result.value
      return scope[ result.value ]
      
    elif result.name == 'assignment':
      name = result.children[0].value
      value = self._eval( result.children[2], scope )
      scope[name] = value
      return Derp.Value( Derp.Value.NULL, None )
      
    elif result.name == 'functionDef':
      name = result.children[0].value
      
      paramList = result.children[2]
      params = [ paramList.children[0].value ]
      for paramListItem in paramList.children[1].children:
        params.append( paramListItem.children[1].value )
        
      body = result.children[5]
      
      scope[name] = Derp.Value( Derp.Value.FUNCTION, ( params, body ) )
      
      return Derp.Value( Derp.Value.NULL, None )
      
    elif result.name == 'functionCall':
      name = result.children[0].value
      assert name in scope, 'unknown function %s' % name
      function = scope[name]
      assert function.type == Derp.Value.FUNCTION
      
      argList = result.children[2]
      argExprs = [ argList.children[0] ]
      for argListItem in argList.children[1].children:
        argExprs.append( argListItem.children[1] )
      args = [ self._eval( argExpr, scope ) for argExpr in argExprs ]
        
      return self._apply( function, args, scope )
      
    elif result.name == 'operationExpr':
      leftExpr = result.children[0].children[0]
      op = result.children[1].value
      rightExpr = result.children[2].children[0]
      
      leftValue = self._eval( leftExpr, scope )
      rightValue = self._eval( rightExpr, scope )
      
      ops = { 
        '+': operator.add,
        '-': operator.sub,
        '*': operator.mul,
        '/': operator.div,
        '==': operator.__eq__,
      }
      assert op in ops
      
      return Derp.Value.fromPy( ops[op]( leftValue.value, rightValue.value ) )
      
    else:
      raise RuntimeError, 'unknown parser result type %s' % result.name

  def _apply( self, function, args, scope ):
    if callable( function.value ):
      return function.value( *args )
    else:
      params, body = function.value
      assert len( params ) == len( args )
      functionScope = copy( scope )
      for param, arg in zip( params, args ):
        functionScope[param] = arg
      return self._eval( body, functionScope )
      
  def eval( self, input ):
    scope = self.defaultScope()
    return [ self._eval( result, scope ) for result in self.parser.parse( input ).children ]
    
  def defaultScope( self ):
    scope = dict( null = Derp.Value( Derp.Value.NULL, None ),
                  true = Derp.Value( Derp.Value.BOOL, True ),
                  false = Derp.Value( Derp.Value.BOOL, False ),
                )
    for name in dir( self ):
      if name.startswith( 'builtin_' ):
        scope[ name[ len( 'builtin_' ): ] ] = Derp.Value( Derp.Value.FUNCTION, getattr(self,name) )
    return scope
    
  def builtin_assert( self, *args ):
    print 'assert', args
    assert all( [ arg.value for arg in args ] ), 'assertion failed: %s' % args

if __name__ == '__main__':
  derp = Derp()
  
  #literals
  assert derp.eval( 'null; true; 1; 1.2; "herro";' ) == [
    Derp.Value( Derp.Value.NULL, None ),
    Derp.Value( Derp.Value.BOOL, True ),
    Derp.Value( Derp.Value.INT, 1 ),
    Derp.Value( Derp.Value.FLOAT, 1.2 ),
    Derp.Value( Derp.Value.STR, 'herro' ),
  ]
  
  #assignment
  assert derp.eval( 'a = -3; a;' ) == [ 
    Derp.Value( Derp.Value.NULL, None ),
    Derp.Value( Derp.Value.INT, -3 ),
  ]
  
  #functions
  assert derp.eval( 'foo( a, b, c ) { c; } foo( 1, 2, 3 );' ) == [
    Derp.Value( Derp.Value.NULL, None ),
    Derp.Value( Derp.Value.INT, 3 ),
  ]
  
  #operators
  assert derp.eval( 'a = 1 + 2; a; 2 == 2; 2 == 3;' ) == [ 
    Derp.Value( Derp.Value.NULL, None ),
    Derp.Value( Derp.Value.INT, 3 ),
    Derp.Value( Derp.Value.BOOL, True ),
    Derp.Value( Derp.Value.BOOL, False ),
  ]
