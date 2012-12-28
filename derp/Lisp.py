from Parser import Parser
from copy import copy

class Lisp:
  
  class Value:
    NULL = 'null'
    INT = 'int'
    FLOAT = 'float'
    STR = 'str'
    FUNCTION = 'function'
  
    def __init__( self, type, value ):
      self.type = type
      self.value = value
    
    def __repr__( self ):
      return 'Lisp.Value( %s, %s )' % ( self.type, self.value )
      
    def __eq__( self, value ):
      return self.type == value.type and self.value == value.value

  def __init__( self ):
    self.parser = Parser.load( '''
      lparen = "\("
      rparen = "\)"
      int = "-?\d+"
      float = "-?\d+\.\d+"
      str = "\"[\S \t]*\""
      id = "\w+"
      ws = ~"\s+"

      exprs = expr*
      expr = id | int | float | str | compoundExpr
      compoundExpr = lparen compoundExprContents rparen
      compoundExprContents = expr+
    ''' )
    
  def _getExpr( self, expr ):
    assert expr.name == 'expr'
    return expr.children[0].name, expr.children[0].value
    
  def _getCompoundExpr( self, expr ):
    assert expr.name == 'expr'
    return expr.children[0].children[1].children
    
  def _eval( self, expr, scope ):
    exprType, exprValue = self._getExpr( expr )
    if exprType == 'id':
      if exprValue == 'null':
        return Lisp.Value( Lisp.Value.NULL, None )
      else:
        assert exprValue in scope, 'unknown var %s' % exprValue
        return scope[exprValue]
    elif exprType == 'int':
      return Lisp.Value( Lisp.Value.INT, int( exprValue ) )
    elif exprType == 'float':
      return Lisp.Value( Lisp.Value.FLOAT, float( exprValue ) )
    elif exprType == 'str':
      return Lisp.Value( Lisp.Value.STR, exprValue[1:-1] )
    elif exprType == 'compoundExpr':
      functionDef = self._getCompoundExpr( expr )
      return self._apply( functionDef[0], functionDef[1:], scope )
      
  def _apply( self, functionExpr, args, scope ):
    function = self._eval( functionExpr, scope )
    if callable( function.value ):
      return function.value( args, scope )
    else:
      params = function.value[0]
      body = function.value[1]
      assert len( params ) == len( args )
      functionScope = copy( scope )
      for param, arg in zip( params, args ):
        functionScope[param] = self._eval( arg, scope )
      return self._eval( body, functionScope )
  
  def eval( self, input ):
    scope = self.defaultScope()
    return [ self._eval( expr, scope ) for expr in self.parser.parse( input ).children ]
    
  def defaultScope( self ):
    prefix = 'builtin_'
    return dict( [ ( name[ len( prefix ): ], Lisp.Value( Lisp.Value.FUNCTION, getattr( self, name ) ) ) for name in dir( self ) if name.startswith( prefix ) ] )
    
  def builtin_add( self, argExprs, scope ):
    args = [ self._eval( argExpr, scope ) for argExpr in argExprs ]
    assert len( args ) == 2
    assert args[0].type == args[1].type
    return Lisp.Value( args[0].type, args[0].value + args[1].value )
    
  def builtin_sub( self, argExprs, scope ):
    args = [ self._eval( argExpr, scope ) for argExpr in argExprs ]
    assert len( args ) == 2
    assert args[0].type == args[1].type
    return Lisp.Value( args[0].type, args[0].value - args[1].value )
    
  def builtin_mul( self, argExprs, scope ):
    args = [ self._eval( argExpr, scope ) for argExpr in argExprs ]
    assert len( args ) == 2
    assert args[0].type == args[1].type
    return Lisp.Value( args[0].type, args[0].value * args[1].value )
    
  def builtin_div( self, argExprs, scope ):
    args = [ self._eval( argExpr, scope ) for argExpr in argExprs ]
    assert len( args ) == 2
    assert args[0].type == args[1].type
    return Lisp.Value( args[0].type, args[0].value / args[1].value )
    
  def builtin_define( self, args, scope ):
    assert len( args ) == 3
    
    nameType, name = self._getExpr( args[0] )
    assert nameType == 'id'
    
    paramsType, paramsValue = self._getExpr( args[1] )
    assert paramsType == 'compoundExpr'
    paramExprs = self._getCompoundExpr( args[1] )
    params = []
    for paramExpr in paramExprs:
      paramType, param = self._getExpr( paramExpr )
      assert paramType == 'id'
      params.append( param )
      
    body = args[2]
    
    scope[name] = Lisp.Value( Lisp.Value.FUNCTION, ( params, body ) )
    
    return Lisp.Value( Lisp.Value.NULL, None )
    
if __name__ == '__main__':
  lisp = Lisp()
  assert lisp.eval( 'null' )[0] == Lisp.Value( Lisp.Value.NULL, None )
  assert lisp.eval( '1' )[0] == Lisp.Value( Lisp.Value.INT, 1 )
  assert lisp.eval( '1.1' )[0] == Lisp.Value( Lisp.Value.FLOAT, 1.1 )
  assert lisp.eval( '"hello world"' )[0] == Lisp.Value( Lisp.Value.STR, "hello world" )
  assert lisp.eval( '( define square ( a ) ( mul a a ) ) (square 2)' )[1] == Lisp.Value( Lisp.Value.INT, 4 )
