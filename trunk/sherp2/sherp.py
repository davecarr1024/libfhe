import re
import types

class Lexer:
    class Result:
        def __init__( self, rule, value ):
            self.rule = rule
            self.value = value
            
        def __repr__( self ):
            return 'Lexer.Result( "%s", "%s" )' % ( self.rule.name, self.value )
            
    class Rule:
        def __init__( self, name, pattern, include ):
            self.name = name
            self.pattern = pattern
            self.include = include
            
        def __repr__( self ):
            return 'Lexer.Rule( "%s", "%s", %s )' % ( self.name, self.pattern, self.include )
            
        def apply( self, input ):
            match = re.match( self.pattern, input )
            if match and match.end() > 0:
                return Lexer.Result( self, input[:match.end()] )
            
    def __init__( self, *rules ):
        self.rules = dict( [ ( rule.name, rule ) for rule in rules ] )
        
    def lex( self, input ):
        pos = 0
        results = []
        while pos < len( input ):
            ruleResults = filter( lambda ( rule, result ): result, 
                [ ( rule, rule.apply( input[pos:] ) ) for rule in self.rules.itervalues() ] )
            assert ruleResults, 'lex error at "%s" %s' % ( input[pos:pos+50], self.rules.keys() )
            rule, result = sorted( ruleResults, key = lambda ( rule, result ): len( result.value ) )[-1]
            if rule.include:
                results.append( result )
            pos += len( result.value )
        return results
        
    @staticmethod
    def test():
        class LexerResult:
            def __init__( self, name, value ):
                self.name = name
                self.value = value
                
            def __repr__( self ):
                return 'Lexer.LexerResult( "%s", "%s" )' % ( self.name, self.value )
                
            def __eq__( self, rhs ):
                assert self.name == rhs.name, ( self.name, rhs.name )
                assert self.value == rhs.value, ( self.value, rhs.value )
                return self.name == rhs.name and self.value == rhs.value
                
            @staticmethod
            def bind( result ):
                return LexerResult( result.rule.name, result.value )
    
        assert [
            LexerResult( 'lparen', '(' ),
            LexerResult( 'id', 'foo' ),
            LexerResult( 'num', '12' ),
            LexerResult( 'str', '"bar"' ),
            LexerResult( 'rparen', ')' ),
        ] == [ LexerResult.bind( result ) for result in Lexer(
            Lexer.Rule( 'lparen', '\(', True ),
            Lexer.Rule( 'rparen', '\)', True ),
            Lexer.Rule( 'str', '\".*?\"', True ),
            Lexer.Rule( 'num', '\d+', True ),
            Lexer.Rule( 'id', '[a-zA-Z_][a-zA-Z0-9_]+', True ),
            Lexer.Rule( 'ws', '\s+', False ),
        ).lex( '( foo 12 "bar")' ) ]

class Parser:
    class Result:
        def __init__( self, rule, value, children ):
            self.rule = rule
            self.value = value
            self.children = children
            
        def __repr__( self, tabs = 0 ):
            s = '%s%s' % ( '    ' * tabs, self.rule.name )
            if self.value:
                s += ' %s' % self.value
            elif self.children:
                s += '\n%s' % '\n'.join( [ child.__repr__( tabs + 1 ) for child in self.children ] )
            return s
            
    class Rule:
        TERMINAL = 'TERMINAL'
        AND = 'AND'
        OR = 'OR'
        ZERO_OR_MORE = 'ZERO_OR_MORE'
        ONE_OR_MORE = 'ONE_OR_MORE'
        ZERO_OR_ONE = 'ZERO_OR_ONE'
        
        def __init__( self, type, name, children ):
            self.type = type
            self.name = name
            self.children = children
            
        def apply( self, pos, tokens ):
            if self.type == Parser.Rule.TERMINAL:
                if pos < len( tokens ) and tokens[ pos ].rule.name == self.name:
                    return pos + 1, Parser.Result( self, tokens[ pos ].value, [] )
            elif self.type == Parser.Rule.AND:
                result = Parser.Result( self, None, [] )
                for child in self.children:
                    childPosResult = child.apply( pos, tokens )
                    if childPosResult:
                        pos, childResult = childPosResult
                        result.children.append( childResult )
                    else:
                        return
                return pos, result
            elif self.type == Parser.Rule.OR:
                for child in self.children:
                    childPosResult = child.apply( pos, tokens )
                    if childPosResult:
                        pos, childResult = childPosResult
                        return pos, Parser.Result( self, None, [ childResult ] )
            elif self.type == Parser.Rule.ZERO_OR_MORE:
                assert len( self.children ) == 1
                result = Parser.Result( self, None, [] )
                while True:
                    childPosResult = self.children[0].apply( pos, tokens )
                    if childPosResult:
                        pos, childResult = childPosResult
                        result.children.append( childResult )
                    else:
                        return pos, result
            elif self.type == Parser.Rule.ONE_OR_MORE:
                assert len( self.children ) == 1
                childPosResult = self.children[0].apply( pos, tokens )
                if childPosResult:
                    pos, childResult = childPosResult
                    result = Parser.Result( self, None, [ childResult ] )
                    while True:
                        childPosResult = self.children[0].apply( pos, tokens )
                        if childPosResult:
                            pos, childResult = childPosResult
                            result.children.append( childResult )
                        else:
                            return pos, result
            elif self.type == Parser.Rule.ZERO_OR_ONE:
                assert len( self.children ) == 1
                childPosResult = self.children[0].apply( pos, tokens )
                if childPosResult:
                    pos, childResult = childPosResult
                    return pos, Parser.Result( self, None, [ childResult ] )
                else:
                    return pos, Parser.Result( self, None, [] )
            else:
                raise NotImplementedError( self.type )
                
    class Exprs:
        class Expr:
            def bind( self, rules, exprs, lexer ):
                raise NotImplementedError()

            @staticmethod
            def parse( result ):
                if result.rule.name in ( 'orMoreArg', 'andOrArg', 'parserRuleExpr' ):
                    return Parser.Exprs.Expr.parse( result.children[0] )
                elif result.rule.name == 'parenRule':
                    return Parser.Exprs.Expr.parse( result.children[1] )
                elif result.rule.name == 'id':
                    return Parser.Exprs.Ref( result.value )
                elif result.rule.name == 'str':
                    return Parser.Exprs.Token( result.value[1:-1] )
                elif result.rule.name == 'zeroOrMoreRule':
                    return Parser.Exprs.Def( None, Parser.Rule.ZERO_OR_MORE, 
                        Parser.Exprs.Expr.parse( result.children[0] ) )
                elif result.rule.name == 'orRule':
                    expr = Parser.Exprs.Def( None, Parser.Rule.OR, 
                        Parser.Exprs.Expr.parse( result.children[0] ) )
                    for child in result.children[1].children:
                        expr.children.append( Parser.Exprs.Expr.parse( child.children[1] ) )
                    return expr
                elif result.rule.name == 'andRule':
                    expr = Parser.Exprs.Def( None, Parser.Rule.AND,
                        Parser.Exprs.Expr.parse( result.children[0] ) )
                    for child in result.children[1].children:
                        expr.children.append( Parser.Exprs.Expr.parse( child ) )
                    return expr
                elif result.rule.name == 'oneOrMoreRule':
                    return Parser.Exprs.Def( None, Parser.Rule.ONE_OR_MORE, Parser.Exprs.Expr.parse( result.children[0] ) )
                elif result.rule.name == 'zeroOrMoreRule':
                    return Parser.Exprs.Def( None, Parser.Rule.ZERO_OR_MORE, Parser.Exprs.Expr.parse( result.children[0] ) )
                elif result.rule.name == 'zeroOrOneRule':
                    return Parser.Exprs.Def( None, Parser.Rule.ZERO_OR_ONE, Parser.Exprs.Expr.parse( result.children[0] ) )
                else:
                    raise NotImplementedError( result )
                
        class Ref( Expr ):
            def __init__( self, value ):
                self.name = None
                self.value = value
                
            def __repr__( self ):
                return 'Parser.Exprs.Ref( "%s" )' % self.value
                
            def bind( self, rules, exprs, lexer ):
                if self.value in rules:
                    return rules[ self.value ]
                elif self.value in exprs:
                    rule = rules[ self.value ] = exprs[ self.value ].bind( rules, exprs, lexer )
                    return rule
                elif self.value in lexer.rules:
                    rule = rules[ self.value ] = Parser.Rule( Parser.Rule.TERMINAL, self.value, [] )
                    return rule
                else:
                    raise RuntimeError( 'unknown ref ' + self.value )
                    
        class Def( Expr ):
            def __init__( self, name, type, *children ):
                self.name = name
                self.type = type
                self.children = list( children )
                
            def __repr__( self ):
                return 'Parser.Exprs.Def( "%s", %s, %s )' % ( self.name, self.type, self.children )
                
            def bind( self, rules, exprs, lexer ):
                if self.name and self.name in rules:
                    return rules[ self.name ]
                else:
                    rule = Parser.Rule( self.type, self.name, [] )
                    if self.name:
                        rules[ self.name ] = rule
                    rule.children = [ child.bind( rules, exprs, lexer ) for child in self.children ]
                    return rule
                    
        class Token( Expr ):
            def __init__( self, name ):
                self.name = name
                
            def __repr__( self ):
                return 'Parser.Exprs.Token( "%s" )' % self.name
                
            def bind( self, rules, exprs, lexer ):
                if self.name in rules:
                    return rules[ self.name ]
                else:
                    if self.name not in lexer.rules:
                        lexer.rules[ self.name ] = Lexer.Rule( self.name, self.name, True )
                    rule = rules[ self.name ] = Parser.Rule( Parser.Rule.TERMINAL, self.name, [] )
                    return rule
                    
    def __init__( self, lexer, root ):
        self.lexer = lexer
        self.root = root
        
    def parse( self, input ):
        tokens = self.lexer.lex( input )
        posResult = self.root.apply( 0, tokens )
        if not posResult:
            raise RuntimeError( 'parse error' )
        else:
            pos, result = posResult
            if pos < len( tokens ):
                raise RuntimeError( 'parse error at %s' % 
                    ' '.join( [ token.value for token in tokens[pos:pos+25] ] ) )
            else:
                return result
        
    @staticmethod
    def bind( lexer, *_exprs ):
        exprs = dict( [ ( expr.name, expr ) for expr in filter( lambda expr: expr.name, _exprs ) ] )
        return Parser( lexer, exprs[_exprs[0].name].bind( {}, exprs, lexer ) )
        
    @staticmethod
    def build( input ):
        grammarParser = Parser.bind( 
            Lexer(
                Lexer.Rule( 'id', '[a-zA-Z_][a-zA-Z0-9_]*', True ),
                Lexer.Rule( 'str', '\'.*?\'', True ),
                Lexer.Rule( 'ws', '\s+', False ),
            ),
            Parser.Exprs.Def( 'grammar', Parser.Rule.ONE_OR_MORE, 
                Parser.Exprs.Def( None, Parser.Rule.OR,
                    Parser.Exprs.Ref( 'lexerRuleDecl' ),
                    Parser.Exprs.Ref( 'negLexerRuleDecl' ),
                    Parser.Exprs.Ref( 'parserRuleDecl' ),
                ),
            ),
            Parser.Exprs.Def( 'lexerRuleDecl', Parser.Rule.AND,
                Parser.Exprs.Ref( 'id' ),
                Parser.Exprs.Token( '=' ),
                Parser.Exprs.Ref( 'str' ),
                Parser.Exprs.Token( ';' ),
            ),
            Parser.Exprs.Def( 'negLexerRuleDecl', Parser.Rule.AND,
                Parser.Exprs.Ref( 'id' ),
                Parser.Exprs.Token( '~=' ),
                Parser.Exprs.Ref( 'str' ),
                Parser.Exprs.Token( ';' ),
            ),
            Parser.Exprs.Def( 'parserRuleDecl', Parser.Rule.AND,
                Parser.Exprs.Ref( 'id' ),
                Parser.Exprs.Token( '=>' ),
                Parser.Exprs.Ref( 'parserRuleExpr' ),
                Parser.Exprs.Token( ';' ),
            ),
            Parser.Exprs.Def( 'parserRuleExpr', Parser.Rule.OR,
                Parser.Exprs.Ref( 'andRule' ),
                Parser.Exprs.Ref( 'orRule' ),
                Parser.Exprs.Ref( 'zeroOrMoreRule' ),
                Parser.Exprs.Ref( 'oneOrMoreRule' ),
                Parser.Exprs.Ref( 'zeroOrOneRule' ),
                Parser.Exprs.Ref( 'str' ),
            ),
            Parser.Exprs.Def( 'andRule', Parser.Rule.AND,
                Parser.Exprs.Ref( 'andOrArg' ),
                Parser.Exprs.Def( None, Parser.Rule.ONE_OR_MORE,
                    Parser.Exprs.Ref( 'andOrArg' ),
                ),
            ),
            Parser.Exprs.Def( 'andOrArg', Parser.Rule.OR,
                Parser.Exprs.Ref( 'zeroOrMoreRule' ),
                Parser.Exprs.Ref( 'oneOrMoreRule' ),
                Parser.Exprs.Ref( 'zeroOrOneRule' ),
                Parser.Exprs.Ref( 'parenRule' ),
                Parser.Exprs.Ref( 'str' ),
                Parser.Exprs.Ref( 'id' ),
            ),
            Parser.Exprs.Def( 'orRule', Parser.Rule.AND,
                Parser.Exprs.Ref( 'andOrArg' ),
                Parser.Exprs.Def( None, Parser.Rule.ONE_OR_MORE,
                    Parser.Exprs.Def( None, Parser.Rule.AND,
                        Parser.Exprs.Token( '\|' ),
                        Parser.Exprs.Ref( 'andOrArg' ),
                    ),
                ),
            ),
            Parser.Exprs.Def( 'orMoreArg', Parser.Rule.OR,
                Parser.Exprs.Ref( 'id' ),
                Parser.Exprs.Ref( 'str' ),
                Parser.Exprs.Ref( 'parenRule' ),
            ),
            Parser.Exprs.Def( 'parenRule', Parser.Rule.AND,
                Parser.Exprs.Token( '\(' ),
                Parser.Exprs.Ref( 'parserRuleExpr' ),
                Parser.Exprs.Token( '\)' ),
            ),
            Parser.Exprs.Def( 'zeroOrMoreRule', Parser.Rule.AND,
                Parser.Exprs.Ref( 'orMoreArg' ),
                Parser.Exprs.Token( '\*' ),
            ),
            Parser.Exprs.Def( 'oneOrMoreRule', Parser.Rule.AND,
                Parser.Exprs.Ref( 'orMoreArg' ),
                Parser.Exprs.Token( '\+' ),
            ),
            Parser.Exprs.Def( 'zeroOrOneRule', Parser.Rule.AND,
                Parser.Exprs.Ref( 'orMoreArg' ),
                Parser.Exprs.Token( '\?' ),
            ),
        )
        
        grammar = grammarParser.parse( input )
        exprs = []
        lexer = Lexer()
        
        for declParent in grammar.children:
            decl = declParent.children[0]
            if decl.rule.name == 'lexerRuleDecl':
                name = decl.children[0].value
                lexer.rules[ name ] = Lexer.Rule( name, decl.children[2].value[1:-1], True )
            elif decl.rule.name == 'negLexerRuleDecl':
                name = decl.children[0].value
                lexer.rules[ name ] = Lexer.Rule( name, decl.children[2].value[1:-1], False )
            elif decl.rule.name == 'parserRuleDecl':
                name = decl.children[0].value
                expr = Parser.Exprs.Expr.parse( decl.children[2].children[0] )
                expr.name = name
                exprs.append( expr )
            else:
                raise NotImplementedError( decl.rule.name )

        return Parser.bind( lexer, *exprs )
        
    @staticmethod
    def test():
        class ParserResult:
            def __init__( self, name, value, *children ):
                self.name = name
                self.value = value
                self.children = list( children )
                
            def __repr__( self, tabs = 0 ):
                s = '%s%s' % ( '    ' * tabs, self.name )
                if self.value:
                    s += ' %s' % self.value
                elif self.children:
                    s += '\n%s' % '\n'.join( [ child.__repr__( tabs + 1 ) for child in self.children ] )
                return s
                
            def __eq__( self, rhs ):
                return self.name == rhs.name and self.value == rhs.value and self.children == rhs.children
                
            @staticmethod
            def bind( result ):
                return ParserResult( result.rule.name, result.value, 
                    *[ ParserResult.bind( child ) for child in result.children ] )
    
        assert ParserResult.bind( Parser.build( """
            num = '\d+';
            id = '[a-zA-Z_][a-zA-Z0-9_]*';
            str = '\".*?\"';
            ws ~= '\s+';
            program => expr+;
            expr => num | id | str | compoundExpr;
            compoundExpr => '\(' expr* '\)';
        """ ).parse( '( foo 12 "bar")' ) ) == \
            ParserResult( 'program', None,
                ParserResult( 'expr', None, 
                    ParserResult( 'compoundExpr', None, 
                        ParserResult( '\(', '(' ),
                        ParserResult( None, None,
                            ParserResult( 'expr', None,
                                ParserResult( 'id', 'foo' ),
                            ),
                            ParserResult( 'expr', None,
                                ParserResult( 'num', '12' ),
                            ),
                            ParserResult( 'expr', None,
                                ParserResult( 'str', '"bar"' ),
                            ),
                        ),
                        ParserResult( '\)', ')' ),
                    ),
                ),
            )

class Lisp:
    class Exprs:
        class Expr:
            def eval( self, scope ):
                raise NotImplementedError()
                
            @staticmethod
            def parse( result ):
                if result.rule.name == 'expr':
                    return Lisp.Exprs.Expr.parse( result.children[0] )
                elif result.rule.name in ( 'id', 'operator' ):
                    return Lisp.Exprs.Ref( result.value )
                elif result.rule.name == 'num':
                    return Lisp.Exprs.Int( int( result.value ) )
                elif result.rule.name == 'str':
                    return Lisp.Exprs.Str( result.value[1:-1] )
                elif result.rule.name == 'compoundExpr':
                    return Lisp.Exprs.Compound( 
                        [ Lisp.Exprs.Expr.parse( child ) for child in result.children[1].children ] )
                else:
                    raise NotImplementedError( result )
                    
        class Ref( Expr ):
            def __init__( self, id ):
                self.id = id
                
            def __repr__( self ):
                return self.id
                
            def eval( self, scope ):
                assert self.id in scope, 'unknown ref %s' % self.id
                return scope[self.id]
                
        class Int( Expr ):
            def __init__( self, value ):
                self.value = value
                
            def __repr__( self ):
                return str( self.value )
                
            def eval( self, scope ):
                return Lisp.Vals.Int( self.value )
                
        class Str( Expr ):
            def __init__( self, value ):
                self.value = value
                
            def __repr__( self ):
                return '"%s"' % self.value
                
            def eval( self, scope ):
                return Lisp.Vals.Str( self.value )
                
        class Compound( Expr ):
            def __init__( self, children ):
                self.children = children
                
            def __repr__( self ):
                return '( %s )' % ' '.join( map( str, self.children ) )
                
            def eval( self, scope ):
                assert len( self.children ) >= 1
                return self.children[0].eval( scope ).apply( self.children[1:], scope )
                
    class Vals:
        class Val:
            def apply( self, args, scope ):
                raise NotImplementedError()
                
        class Bool( Val ):
            def __init__( self, value ):
                self.value = value
                
            def __eq__( self, rhs ):
                return self.value == rhs.value
                
        class Int( Val ):
            def __init__( self, value ):
                self.value = value
                
            def __eq__( self, rhs ):
                return self.value == rhs.value
                
        class Str( Val ):
            def __init__( self, value ):
                self.value = value
                
            def __eq__( self, rhs ):
                return self.value == rhs.value
                
        class Builtin( Val ):
            def __init__( self, func ):
                self.func = func
                
            def apply( self, args, scope ):
                return self.func( args, scope )
                
        class Func( Val ):
            def __init__( self, params, body ):
                self.params = params
                self.body = body
                
            def apply( self, args, scope ):
                assert len( args ) == len( self.params )
                funcScope = dict( [ ( key, val ) for key, val in scope.iteritems() ] )
                for arg, param in zip( args, self.params ):
                    funcScope[param] = arg.eval( scope )
                return [ expr.eval( funcScope ) for expr in self.body ][-1]
                
    class Builtins:
        @staticmethod
        def add( args, scope ):
            vals = [ arg.eval( scope ) for arg in args ]
            if len( vals ) == 2 and isinstance( vals[0], Lisp.Vals.Int ) and isinstance( vals[1], Lisp.Vals.Int ):
                return Lisp.Vals.Int( vals[0].value + vals[1].value )
            else:
                raise NotImplementedError( vals )
                
        @staticmethod
        def sub( args, scope ):
            vals = [ arg.eval( scope ) for arg in args ]
            if len( vals ) == 2 and isinstance( vals[0], Lisp.Vals.Int ) and isinstance( vals[1], Lisp.Vals.Int ):
                return Lisp.Vals.Int( vals[0].value - vals[1].value )
            else:
                raise NotImplementedError( vals )
            
        @staticmethod
        def mul( args, scope ):
            vals = [ arg.eval( scope ) for arg in args ]
            if len( vals ) == 2 and isinstance( vals[0], Lisp.Vals.Int ) and isinstance( vals[1], Lisp.Vals.Int ):
                return Lisp.Vals.Int( vals[0].value * vals[1].value )
            else:
                raise NotImplementedError( vals )
            
        @staticmethod
        def div( args, scope ):
            vals = [ arg.eval( scope ) for arg in args ]
            if len( vals ) == 2 and isinstance( vals[0], Lisp.Vals.Int ) and isinstance( vals[1], Lisp.Vals.Int ):
                return Lisp.Vals.Int( vals[0].value / vals[1].value )
            else:
                raise NotImplementedError( vals )
                
        @staticmethod
        def define( args, scope ):
            assert isinstance( args[0], Lisp.Exprs.Ref )
            if len( args ) == 2:
                val = scope[args[0].id] = args[1].eval( scope )
                return val
            elif len( args ) >= 3 and isinstance( args[1], Lisp.Exprs.Compound ) \
                and all( [ isinstance( arg, Lisp.Exprs.Ref ) for arg in args[1].children ] ):
                val = scope[args[0].id] = Lisp.Vals.Func( [ arg.id for arg in args[1].children ], args[2:] )
                return val
            else:
                raise NotImplementedError( args )
                        
    parser = Parser.build( """
        num = '[-]?\d+';
        id = '[a-zA-Z_][a-zA-Z0-9_]*';
        operator = '[\+\-\*\/]';
        str = '\".*?\"';
        ws ~= '\s+';
        program => expr+;
        expr => num | id | operator | str | compoundExpr;
        compoundExpr => '\(' expr* '\)';
    """ )
                
    @staticmethod
    def eval( input ):
        scope = {
            'true': Lisp.Vals.Bool( True ),
            'false': Lisp.Vals.Bool( False ),
            '+': Lisp.Vals.Builtin( Lisp.Builtins.add ),
            '-': Lisp.Vals.Builtin( Lisp.Builtins.sub ),
            '*': Lisp.Vals.Builtin( Lisp.Builtins.mul ),
            '/': Lisp.Vals.Builtin( Lisp.Builtins.div ),
            'define': Lisp.Vals.Builtin( Lisp.Builtins.define ),
        }
        return [ Lisp.Exprs.Expr.parse( result ).eval( scope ) for result in Lisp.parser.parse( input ).children ][-1]
        
    @staticmethod
    def test():
        assert Lisp.Vals.Bool( True ) == Lisp.eval( 'true' )
        assert Lisp.Vals.Bool( False ) == Lisp.eval( 'false' )
        assert Lisp.Vals.Int( 3 ) == Lisp.eval( '3' )
        assert Lisp.Vals.Int( -12 ) == Lisp.eval( '-12' )
        assert Lisp.Vals.Int( 'foobar' ) == Lisp.eval( '"foobar"' )
        assert Lisp.Vals.Int( 7 ) == Lisp.eval( '(+ 3 4)' )
        assert Lisp.Vals.Int( 1 ) == Lisp.eval( '(- 3 2)' )
        assert Lisp.Vals.Int( 6 ) == Lisp.eval( '(* 3 2)' )
        assert Lisp.Vals.Int( 5 ) == Lisp.eval( '(/ 10 2)' )
        assert Lisp.Vals.Int( 3 ) == Lisp.eval( '(define foo 2) (+ foo 1)' )
        assert Lisp.Vals.Int( 20 ) == Lisp.eval( '( define foo ( x ) ( * x 2 ) ) ( foo 10 )' )

class Sharpy:
    class Exprs:
        class Expr:
            def eval( self, scope ):
                raise NotImplementedError()
                
            @staticmethod
            def parse( result ):
                if result.rule.name in ( 'statement', 'exprStatement', 'expr', 'operand' ):
                    return Sharpy.Exprs.Expr.parse( result.children[0] )
                elif result.rule.name == 'ref':
                    #ref => id ( '\.' id )*;
                    return Sharpy.Exprs.Ref( result.children[0].value, *[ child.children[1].value for child in result.children[1].children ] )
                elif result.rule.name == 'int':
                    return Sharpy.Exprs.Int( int( result.value ) )
                elif result.rule.name == 'str':
                    return Sharpy.Exprs.Str( result.value[1:-1] )
                elif result.rule.name == 'call':
                    #call => ref '\(' ( expr ( ',' expr )* )? '\)';
                    if result.children[2].children:
                        argExpr = result.children[2].children[0]
                        args = [ Sharpy.Exprs.Expr.parse( argExpr.children[0] ) ] + \
                            [ Sharpy.Exprs.Expr.parse( child.children[1] ) for child in argExpr.children[1].children ]
                    else:
                        args = []
                    return Sharpy.Exprs.Call( Sharpy.Exprs.Expr.parse( result.children[0] ), args )
                elif result.rule.name == 'binaryOperation':
                    #binaryOperation => operand binaryOperator operand;
                    return Sharpy.Exprs.BinaryOperation( 
                        result.children[1].children[0].value, 
                        Sharpy.Exprs.Expr.parse( result.children[0] ),
                        Sharpy.Exprs.Expr.parse( result.children[2] ) 
                    )
                elif result.rule.name == 'unaryOperation':
                    #unaryOperation => unaryOperator operand;
                    return Sharpy.Exprs.UnaryOperation(
                        result.children[0].children[0].value,
                        Sharpy.Exprs.Expr.parse( result.children[1] )
                    )
                elif result.rule.name == 'funcDecl':
                    #funcDecl => 'def' id '\(' ( id ( ',' id )* )? '\)' '{' statement* '}';
                    name = result.children[1].value
                    paramExpr = result.children[3].children[0]
                    if paramExpr:
                        params = [ paramExpr.children[0].value ] + \
                            [ child.children[1].value for child in paramExpr.children[1].children ]
                    else:
                        params = []
                    body = map( Sharpy.Exprs.Expr.parse, result.children[6].children )
                    return Sharpy.Exprs.Func( name, params, body )
                elif result.rule.name == 'returnStatement':
                    return Sharpy.Exprs.ReturnStatement( Sharpy.Exprs.Expr.parse( result.children[1] ) )
                elif result.rule.name == 'parenExpr':
                    return Sharpy.Exprs.Expr.parse( result.children[1] )
                elif result.rule.name == 'classDecl':
                    #classDecl => 'class' id '{' statement* '}';
                    return Sharpy.Exprs.Class( result.children[1].value,
                        map( Sharpy.Exprs.Expr.parse, result.children[3].children ) )
                elif result.rule.name == 'ifStatement':
                    #ifStatement => 'if' '\(' expr '\)' '{' statement* '}';
                    return Sharpy.Exprs.IfStatement( 
                        Sharpy.Exprs.Expr.parse( result.children[2] ),
                        map( Sharpy.Exprs.Expr.parse, result.children[5].children ),
                        None 
                    )
                elif result.rule.name == 'ifElseStatement':
                    #ifElseStatement => 'if' '\(' expr '\)' '{' statement* '}' 'else' '{' statement* '}';
                    return Sharpy.Exprs.IfStatement(
                        Sharpy.Exprs.Expr.parse( result.children[2] ),
                        map( Sharpy.Exprs.Expr.parse, result.children[5].children ),
                        map( Sharpy.Exprs.Expr.parse, result.children[9].children )
                    )
                elif result.rule.name == 'whileStatement':
                    #whileStatement => 'while' '\(' expr '\)' '{' statement* '}';
                    return Sharpy.Exprs.WhileStatement(
                        Sharpy.Exprs.Expr.parse( result.children[2] ),
                        map( Sharpy.Exprs.Expr.parse, result.children[5].children )
                    )
                elif result.rule.name == 'forStatement':
                    #forStatement => 'for' '\(' expr ';' expr ';' expr '\)' '{' statement* '}';
                    return Sharpy.Exprs.ForStatement(
                        Sharpy.Exprs.Expr.parse( result.children[2] ),
                        Sharpy.Exprs.Expr.parse( result.children[4] ),
                        Sharpy.Exprs.Expr.parse( result.children[6] ),
                        map( Sharpy.Exprs.Expr.parse, result.children[9].children )
                    )
                else:
                    raise NotImplementedError( result )
                    
        class Ref( Expr ):
            def __init__( self, id, *ids ):
                self.ids = [ id ] + list( ids )
                
            def __repr__( self ):
                return '.'.join( self.ids )
                
            def resolve( self, scope ):
                for id in self.ids[:-1]:
                    scope = scope[id].getScope()
                    assert scope, 'trying to get member from non-scope id %s in ref %s' % ( id, self )
                return scope
                
            def eval( self, scope ):
                return self.resolve( scope )[self.ids[-1]]
                
        class Int( Expr ):
            def __init__( self, value ):
                self.value = value
                
            def __repr__( self ):
                return str( self.value )
                
            def eval( self, scope ):
                return Sharpy.Vals.Int( self.value )
                
        class Str( Expr ):
            def __init__( self, value ):
                self.value = value
                
            def __repr__( self ):
                return '"%s"' % self.value
                
            def eval( self, scope ):
                return Sharpy.Vals.Str( self.value )
                
        class Call( Expr ):
            def __init__( self, func, args ):
                self.func = func
                self.args = args
                
            def __repr__( self ):
                return '%s(%s)' % ( self.func, ', '.join( map( str, self.args ) ) )
                
            def eval( self, scope ):
                func = self.func.eval( scope )
                assert func.canApply(), 'trying to call noncallable %s' % func
                return func.apply( self.args, scope )
                
        class Val( Expr ):
            def __init__( self, value ):
                self.value = value
                
            def eval( self, scope ):
                return self.value
                
        class UnaryOperation( Expr ):
            NOT = 'NOT'
            INC = 'INC'
            DEC = 'DEC'
            STR_TO_OP = { '!': NOT, '++': INC, '--': DEC, }
            OP_TO_STR = { NOT: '!', INC: '++', DEC: '--', }
            OP_TO_FUNC = { NOT: '__not__', INC: '__inc__', DEC: '__dec__', }
            
            def __init__( self, op, rhs ):
                assert op in Sharpy.Exprs.UnaryOperation.STR_TO_OP, op
                self.op = Sharpy.Exprs.UnaryOperation.STR_TO_OP[ op ]
                self.rhs = rhs
                
            def __repr__( self ):
                return '%s %s' % ( Sharpy.Exprs.UnaryOperation.OP_TO_STR[ self.op ], self.rhs )
                
            def eval( self, scope ):
                rhs = self.rhs.eval( scope )
                rhsScope = rhs.getScope()
                assert rhsScope
                funcName = Sharpy.Exprs.UnaryOperation.OP_TO_FUNC[ self.op ]
                assert funcName in rhsScope, "obj %s doesn't implement op %s with func %s" % \
                    ( rhs, Sharpy.Exprs.UnaryOperation.OP_TO_STR[ self.op ], funcName )
                func = rhsScope[ funcName ]
                assert func.canApply()
                return func.apply( [], scope )
                
        class BinaryOperation( Expr ):
            ASSIGN = 'ASSIGN'
            EQUALS = 'EQUALS'
            ADD = 'ADD'
            SUB = 'SUB'
            MUL = 'MUL'
            DIV = 'DIV'
            LT = 'LT'
            LTE = 'LTE'
            GT = 'GT'
            GTE = 'GTE'
            IADD = 'IADD'
            ISUB = 'ISUB'
            IMUL = 'IMUL'
            IDIV = 'IDIV'
            STR_TO_OP = { '=': ASSIGN, '==': EQUALS, '+': ADD, '-': SUB, '*': MUL, '/': DIV, '<': LT, '<=': LTE, '>': GT, '>=': GTE, '+=': IADD, '-=': ISUB, '*=': IMUL, '/=': IDIV, }
            OP_TO_STR = { ASSIGN: '=', EQUALS: '==', ADD: '+', SUB: '-', MUL: '*', DIV: '/', LT: '<', LTE: '<=', GT: '>', GTE: '>=', IADD: '+=', ISUB: '-=', IMUL: '*=', IDIV: '/=', }
            OP_TO_FUNC = { EQUALS: '__eq__', ADD: '__add__', SUB: '__sub__', MUL: '__mul__', DIV: '__div__', LT: '__lt__', LTE: '__lte__', GT: '__gt__', GTE: '__gte__', IADD: '__iadd__', ISUB: '__isub__', IMUL: '__imul__', IDIV: '__idiv__', }
            
            def __init__( self, op, lhs, rhs ):
                assert op in Sharpy.Exprs.BinaryOperation.STR_TO_OP, op
                self.op = Sharpy.Exprs.BinaryOperation.STR_TO_OP[op]
                self.lhs = lhs
                self.rhs = rhs
                
            def __repr__( self ):
                return '%s %s %s' % ( self.lhs, Sharpy.Exprs.BinaryOperation.OP_TO_STR[ self.op ], self.rhs )
                
            def eval( self, scope ):
                if self.op == Sharpy.Exprs.BinaryOperation.ASSIGN:
                    assert isinstance( self.lhs, Sharpy.Exprs.Ref )
                    lhsScope = self.lhs.resolve( scope )
                    val = lhsScope[ self.lhs.ids[-1] ] = self.rhs.eval( scope )
                    return val
                else:
                    lhs = self.lhs.eval( scope )
                    lhsScope = lhs.getScope()
                    assert lhsScope
                    funcName = Sharpy.Exprs.BinaryOperation.OP_TO_FUNC[ self.op ]
                    assert funcName in lhsScope, "obj %s doesn't implement op %s with func %s" % \
                        ( lhs, Sharpy.Exprs.BinaryOperation.OP_TO_STR[ self.op ], funcName )
                    func = lhsScope[ funcName ]
                    assert func.canApply()
                    return func.apply( [ self.rhs ], scope )
                
        class ReturnStatement( Expr ):
            def __init__( self, value ):
                self.value = value
                
            def __repr__( self ):
                return 'return %s' % self.value
                
            def eval( self, scope ):
                value = self.value.eval( scope )
                value.isReturn = True
                return value
                
        class Func( Expr ):
            def __init__( self, name, params, body ):
                self.name = name
                self.params = params
                self.body = body
                
            def eval( self, scope ):
                val = scope[self.name] = Sharpy.Vals.Func( self.name, self.params, self.body, scope )
                return val
                
        class Class( Expr ):
            def __init__( self, name, body ):
                self.name = name
                self.body = body
                
            def eval( self, scope ):
                classScope = Sharpy.Scope( scope )
                for expr in self.body:
                    expr.eval( classScope )
                c = scope[self.name] = Sharpy.Vals.Class( self.name, classScope )
                return c
                
        class IfStatement( Expr ):
            def __init__( self, cond, posBody, negBody ):
                self.cond = cond
                self.posBody = posBody
                self.negBody = negBody
                
            def __repr__( self ):
                if self.negBody:
                    return 'if ( %s ) { %s } else { %s }' % \
                        ( self.cond, '; '.join( map( str, self.posBody ) ), '; '.join( map( str, self.negBody ) ) )
                else:
                    return 'if ( %s ) { %s }' % ( self.cond, '; '.join( self.posBody ) )
                    
            def eval( self, scope ):
                if Sharpy.Vals.Bool( self.cond.eval( scope ) ):
                    body = self.posBody
                else:
                    body = self.negBody
                if body:
                    for expr in body:
                        val = expr.eval( scope )
                        if val.isReturn:
                            return val
                return Sharpy.Vals.NoneType()
                
        class WhileStatement( Expr ):
            def __init__( self, cond, body ):
                self.cond = cond
                self.body = body
                
            def __repr__( self ):
                return 'while ( %s ) { %s }' % ( self.cond, '; '.join( map( str, self.body ) ) )
                
            def eval( self, scope ):
                while Sharpy.Vals.Bool( self.cond.eval( scope ) ).value:
                    for expr in self.body:
                        val = expr.eval( scope )
                        if val.isReturn:
                            return val
                return Sharpy.Vals.NoneType()
                
        class ForStatement( Expr ):
            def __init__( self, init, cond, iter, body ):
                self.init = init
                self.cond = cond
                self.iter = iter
                self.body = body
                
            def __repr__( self ):
                return 'for ( %s; %s; %s ) { %s }' % \
                    ( self.init, self.cond, self.iter, '; '.join( map( str, self.body ) ) )
                    
            def eval( self, scope ):
                self.init.eval( scope )
                while Sharpy.Vals.Bool( self.cond.eval( scope ) ).value:
                    for expr in self.body:
                        val = expr.eval( scope )
                        if val.isReturn:
                            return val
                    self.iter.eval( scope )
                return Sharpy.Vals.NoneType()
                    
    class Vals:
        class Val:
            def __init__( self ):
                self.isReturn = False
        
            def canApply( self ):
                return False
        
            def apply( self, args, scope ):
                raise NotImplementedError()
                
            def getScope( self ):
                return None
                
        class Object( Val ):
            def __init__( self, type ):
                Sharpy.Vals.Val.__init__( self )
                self.type = type
                self.scope = Sharpy.Scope( type.getScope() )
                for name, val in self.scope.getVals().iteritems():
                    if val.canApply():
                        self.scope[name] = Sharpy.Vals.Method( self, val )
                
            def __repr__( self ):
                return 'Object(%s)' % self.type
                
            def getScope( self ):
                return self.scope
                
        class Class( Val ):
            def __init__( self, name, scope ):
                Sharpy.Vals.Val.__init__( self )
                self.name = name
                self.scope = scope
                
            def __repr__( self ):
                return self.name
                
            def canApply( self ):
                return True
                
            def getScope( self ):
                return self.scope
                
            def apply( self, args, scope ):
                obj = Sharpy.Vals.Object( self )
                if '__init__' in obj.scope:
                    obj.scope['__init__'].apply( args, scope )
                return obj
                
        class Method( Val ):
            def __init__( self, obj, func ):
                self.obj = obj
                self.func = func
                
            def canApply( self ):
                return True
                
            def apply( self, args, scope ):
                return self.func.apply( [ Sharpy.Exprs.Val( self.obj ) ] + args, scope )
                
        class BuiltinFunc( Val ):
            def __init__( self, func ):
                Sharpy.Vals.Val.__init__( self )
                self.func = func
                
            def canApply( self ):
                return True
                
            def apply( self, args, scope ):
                return self.func( args, scope )
                
        class BuiltinClass( Val ):
            BUILTINS = {}
            SYSPREFIX = 'sys_'
        
            def __init__( self, type ):
                Sharpy.Vals.Val.__init__( self )
                self.type = type
                self.scope = Sharpy.Scope( None )
                for name, val in [ ( name, getattr( type, name ) ) for name in dir( type ) ]:
                    if callable( val ):
                        prefix = Sharpy.Vals.BuiltinClass.SYSPREFIX
                        if name.startswith( prefix ):
                            def bind( type, name, func ):
                                def call( args, scope ):
                                    val = func( args, scope )
                                    if val == None:
                                        return Sharpy.Vals.NoneType()
                                    else:
                                        return val
                                return call
                            self.scope[ name[ len(prefix): ] ] = Sharpy.Vals.BuiltinFunc( bind( self.type, name, val ) )
                        else:
                            def bind( type, name, func ):
                                def call( args, scope ):
                                    vals = [ arg.eval( scope ) for arg in args ]
                                    val = func( *vals )
                                    if val == None:
                                        return Sharpy.Vals.NoneType()
                                    else:
                                        return val
                                return call
                            self.scope[ name ] = Sharpy.Vals.BuiltinFunc( bind( self.type, name, val ) )
                    
            def __repr__( self ):
                return self.type.__name__
                    
            def canApply( self ):
                return True
                
            def getScope( self ):
                return self.scope
                
            def apply( self, args, scope ):
                return self.type( *[ arg.eval( scope ) for arg in args ] )
                
            @staticmethod
            def bind( type ):
                if type not in Sharpy.Vals.BuiltinClass.BUILTINS:
                    Sharpy.Vals.BuiltinClass.BUILTINS[ type ] = Sharpy.Vals.BuiltinClass( type )
                return Sharpy.Vals.BuiltinClass.BUILTINS[ type ]
                
        class NoneType( Object ):
            def __init__( self ):
                Sharpy.Vals.Object.__init__( self,  Sharpy.Vals.BuiltinClass.bind( Sharpy.Vals.NoneType ) )
                assert not self.isReturn
                
            def __eq__( self, rhs ):
                return isinstance( rhs, Sharpy.Vals.NoneType )
                
            def __repr__( self ):
                return 'None'
                
        class Bool( Object ):
            def __init__( self, value ):
                Sharpy.Vals.Object.__init__( self, Sharpy.Vals.BuiltinClass.bind( Sharpy.Vals.Bool ) )
                if isinstance( value, Sharpy.Vals.Bool ):
                    self.value = value.value
                elif type( value ) == bool:
                    self.value = value
                else:
                    raise NotImplementedError( type( value ) )
                    
            def __eq__( self, rhs ):
                return isinstance( rhs, Sharpy.Vals.Bool ) and self.value == rhs.value
                
            def __repr__( self ):
                return str( self.value )
                
            def __not__( self ):
                return Sharpy.Vals.Bool( not self.value )
                
        class Int( Object ):
            def __init__( self, value ):
                Sharpy.Vals.Object.__init__( self,  Sharpy.Vals.BuiltinClass.bind( Sharpy.Vals.Int ) )
                if isinstance( value, Sharpy.Vals.Int ):
                    self.value = value.value
                elif type( value ) == int:
                    self.value = value
                else:
                    raise NotImplementedError( type( value ) )
                    
            def __eq__( self, rhs ):
                return isinstance( rhs, Sharpy.Vals.Int ) and self.value == rhs.value
                
            def __repr__( self ):
                return str( self.value )
                
            def __add__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    return Sharpy.Vals.Int( self.value + rhs.value )
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __sub__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    return Sharpy.Vals.Int( self.value - rhs.value )
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __mul__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    return Sharpy.Vals.Int( self.value * rhs.value )
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __div__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    return Sharpy.Vals.Int( self.value / rhs.value )
                else:
                    raise NotImplementedError( type( rhs ) )
                    
            def __inc__( self ):
                self.value += 1
                return Sharpy.Vals.Int( self )
                
            def __dec__( self ):
                self.value -= 1
                return Sharpy.Vals.Int( self.value )
                
            def __iadd__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    self.value += rhs.value
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __isub__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    self.value -= rhs.value
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __imul__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    self.value *= rhs.value
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __idiv__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    self.value /= rhs.value
                else:
                    raise NotImplementedError( type( rhs ) )
                    
            def __lt__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    return Sharpy.Vals.Bool( self.value < rhs.value )
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __lte__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    return Sharpy.Vals.Bool( self.value <= rhs.value )
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __gt__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    return Sharpy.Vals.Bool( self.value > rhs.value )
                else:
                    raise NotImplementedError( type( rhs ) )
                
            def __gte__( self, rhs ):
                if isinstance( rhs, Sharpy.Vals.Int ):
                    return Sharpy.Vals.Bool( self.value >= rhs.value )
                else:
                    raise NotImplementedError( type( rhs ) )
                
        class Str( Object ):
            def __init__( self, value ):
                Sharpy.Vals.Object.__init__( self,  Sharpy.Vals.BuiltinClass.bind( Sharpy.Vals.Str ) )
                if isinstance( value, Sharpy.Vals.Str ):
                    self.value = value.value
                elif type( value ) == str:
                    self.value = value
                else:
                    raise NotImplementedError( type( value ) )
                    
            def __eq__( self, rhs ):
                return isinstance( rhs, Sharpy.Vals.Str ) and self.value == rhs.value
                
            def __repr__( self ):
                return '"%s"' % self.value
                
        class System( Object ):
            def __init__( self ):
                Sharpy.Vals.Object.__init__( self, Sharpy.Vals.BuiltinClass.bind( Sharpy.Vals.System ) )
                
            @staticmethod
            def sys_Assert( args, scope ):
                for arg in args:
                    assert Sharpy.Vals.Bool( arg.eval( scope ) ), 'assertion %s failed' % arg
                    
            @staticmethod
            def Print( *vals ):
                print ' '.join( map( str, vals ) )
                return Sharpy.Vals.NoneType()
                
        class Func( Val ):
            def __init__( self, name, params, body, scope ):
                self.name = name
                self.params = params
                self.body = body
                self.scope = scope
                
            def canApply( self ):
                return True
                
            def apply( self, args, scope ):
                assert len( self.params ) == len( args ), ( self.params, args )
                vals = [ arg.eval( scope ) for arg in args ]
                funcScope = Sharpy.Scope( self.scope )
                for name, val in zip( self.params, vals ):
                    funcScope[name] = val
                for expr in self.body:
                    val = expr.eval( funcScope )
                    if val.isReturn:
                        return val
                
    class Scope:
        def __init__( self, parent ):
            self.vals = dict()
            self.parent = parent
            
        def __getitem__( self, id ):
            if id in self.vals:
                return self.vals[id]
            elif self.parent:
                return self.parent[ id ]
            else:
                raise RuntimeError( 'unknown id %s' % id )
                
        def __setitem__( self, id, val ):
            self.vals[id] = val
            
        def __contains__( self, id ):
            return id in self.vals or ( self.parent and id in self.parent )
            
        def __repr__( self ):
            return str( self.getVals() )
            
        def getVals( self ):
            if self.parent:
                vals = self.parent.getVals()
            else:
                vals = {}
            vals.update( self.vals )
            return vals
                
    parser = Parser.build( """
        int = '[-]?\d+';
        str = '".*?"';
        id = '[a-zA-Z_][a-zA-Z0-9_]*';
        ws ~= '\s+';
        program => statement+;
        statement => exprStatement | returnStatement | funcDecl | classDecl | ifElseStatement | ifStatement | whileStatement | forStatement;
        exprStatement => expr ';';
        returnStatement => 'return' expr ';';
        expr => unaryOperation | binaryOperation | parenExpr | call | ref | str | int;
        ref => id ( '\.' id )*;
        call => ref '\(' ( expr ( ',' expr )* )? '\)';
        funcDecl => 'def' id '\(' ( id ( ',' id )* )? '\)' '{' statement* '}';
        binaryOperation => operand binaryOperator operand;
        binaryOperator => '=' | '==' | '\+' | '\-' | '\*' | '\/' | '<' | '<=' | '>' | '>=' | '\+=' | '\-=' | '\*=' | '\/=';
        operand => call | ref | parenExpr | str | int;
        unaryOperation => unaryOperator operand;
        unaryOperator => '!' | '\+\+' | '--';
        parenExpr => '\(' expr '\)';
        classDecl => 'class' id '{' statement* '}';
        ifStatement => 'if' '\(' expr '\)' '{' statement* '}';
        ifElseStatement => 'if' '\(' expr '\)' '{' statement* '}' 'else' '{' statement* '}';
        whileStatement => 'while' '\(' expr '\)' '{' statement* '}';
        forStatement => 'for' '\(' expr ';' expr ';' expr '\)' '{' statement* '}';
    """ )
    
    scope = None
    
    @staticmethod
    def defaultScope():
        if not Sharpy.scope:
            Sharpy.scope = Sharpy.Scope( None )
            Sharpy.scope['None'] = Sharpy.Vals.NoneType()
            Sharpy.scope['False'] = Sharpy.Vals.Bool( False )
            Sharpy.scope['True'] = Sharpy.Vals.Bool( True )
            for name, type in filter( lambda ( name, type ): not name.startswith( "_" ) and isinstance( type, types.ClassType ), Sharpy.Vals.__dict__.iteritems() ):
                Sharpy.scope[ name ] = Sharpy.Vals.BuiltinClass.bind( type )
        return Sharpy.scope
            
    @staticmethod
    def eval( input, scope = None ):
        scope = scope or Sharpy.defaultScope()
        return [ Sharpy.Exprs.Expr.parse( child ).eval( scope ) for child in Sharpy.parser.parse( input ).children ][-1]

    @staticmethod
    def test():
        Sharpy.eval( """
            System.Assert( None == NoneType() );
            System.Assert( False == Bool( False ) );
            System.Assert( True == Bool( True ) );
            System.Assert( -13 == Int( -13 ) );
            System.Assert( "foo" == Str( "foo" ) );
            
            a = 3;
            System.Assert( a == 3 );
            
            System.Assert( ( 3 + 2 ) == 5 );
            System.Assert( ( 13 - 2 ) == 11 );
            System.Assert( ( 5 * 3 ) == 15 );
            System.Assert( ( 25 / 5 ) == 5 );
            
            a = 0;
            ++a;
            System.Assert( a == 1 );
            --a;
            System.Assert( a == 0 );
            
            System.Assert( !False );
            
            a = 3;
            a += 1;
            System.Assert( a == 4 );
            a -= 2;
            System.Assert( a == 2 );
            a *= 5;
            System.Assert( a == 10 );
            a /= 2;
            System.Assert( a == 5 );
            
            def testAssign( a, b )
            {
                c = ( a * 2 );
                return c + b;
            }
            System.Assert( 11 == testAssign( 3, 5 ) );

            class testClass
            {
                def __init__( self, a )
                {
                    self.a = a;
                }
                def bar( self, b )
                {
                    return self.a * b;
                }
            }
            f = testClass( 15 );
            System.Assert( 30 == f.bar( 2 ) );
            
            def testIf( a, b )
            {
                if ( a == "foo" )
                {
                    return b * 2;
                }
                return b;
            }
            System.Assert( testIf( "foo", 10 ) == 20 );
            System.Assert( testIf( "bar", 10 ) == 10 );
            
            def testIfElse( a )
            {
                if ( a == 3 )
                {
                    return "foo";
                }
                else
                {
                    return "bar";
                }
            }
            System.Assert( testIfElse( 3 ) == "foo" );
            System.Assert( testIfElse( 4 ) == "bar" );
            
            def testWhile( a, b )
            {
                p = 1;
                i = 0;
                while ( i < a )
                {
                    i += 1;
                    p *= b;
                }
                return b;
            }
            System.Assert( testWhile( 3, 2 ) == 8 );
            
            def testFor( n, s )
            {
                for ( i = 0; i < n; ++i )
                {
                    s += s;
                }
                return s;
            }
            System.Assert( testFor( 3, 2 ) == 6 );
        """ )
        
class SharpBuiltin:
    def __init__( self, static, retType, *argTypes ):
        self.static = static
        bin = Sharp.Vals.BuiltinClass.bind
        self.retType = bind( retType )
        self.argTypes = map( bind, argTypes )
        
    def __call__( self, func ):
        self.func = func
        
class Sharp:
    class Exprs:
        class Expr:
            def eval( self, scope ):
                raise NotImplementedError()
                
            @staticmethod
            def parse( result ):
                parse = Sharp.Exprs.Expr.parse
                if result.rule.name in ( 'statement', 'exprStatement', 'expr' ):
                    return parse( result.children[0] )
                elif result.rule.name == 'ref':
                    #ref => id ( '\.' id )*;
                    return Sharp.Exprs.Ref( [ result.children[0].value ] + \
                        [ child.children[1].value for child in result.children[1].children ] )
                else:
                    raise NotImplementedError( result )
                
        class Ref:
            def __init__( self, ids ):
                self.ids = ids
                
            def __repr__( self ):
                return '.'.join( self.ids )
                
            def eval( self, scope ):
                for id in self.ids[:-1]:
                    scope = scope.get( id ).val.scope
                    assert scope, id
                return scope.get( self.ids[-1] ).val
                
    class Vals:
        class Val:
            def __init__( self ):
                self.isReturn = False
                
            def getScope( self ):
                return None
                
            def canApply( self, args ):
                return False
                
            def apply( self, args ):
                raise NotImplementedError()
                
        class BuiltinClass( Val ):
            builtins = {}
        
            def __init__( self, type ):
                Sharp.Vals.Val.__init__( self )
                self.type = type
                self.scope = Sharp.Scope( None )
                
            def getScope( self ):
                return self.scope
                
            def canApply( self, args ):
                return self.scope.canApply( '__init__', args )
                
            def apply( self, args ):
                assert 0
                
            @staticmethod
            def bind( type ):
                if not type in Sharp.Vals.BuiltinClass.builtins:
                    Sharp.Vals.BuiltinClass.builtins[ type ] = Sharp.Vals.BuiltinClass( type )
                return Sharp.Vals.BuiltinClass.builtins[ type ]
                
        class Object( Val ):
            def __init__( self, type ):
                Sharp.Vals.Val.__init__( self )
                self.type = type
                self.scope = Sharp.Scope( self.type.getScope() )
                for expr in self.type.dynamicBody:
                    expr.eval( self.scope )
                    
            def getScope( self ):
                return self.scope
                
            def canApply( self, args ):
                return self.scope.canApply( '__call__', args )
                
            def apply( self, args ):
                return self.scope.apply( '__call__', args )
                
    class Builtins:
        pass
                
    class Var:
        def __init__( self, type, name, val ):
            self.type = type
            self.name = name
            self.val = val
            
    class Scope:
        def __init__( self, parent ):
            self.parent = parent
            self.vars = []
            
        def getAll( self, id ):
            return filter( lambda var: var.name == id, self.vars )
            
        def get( self, id ):
            vars = self.getAll( id )
            if len( vars ):
                raise RuntimeError( 'ambiguous id %s' % id )
            elif len( vars ) == 1:
                return vars[0]
            elif self.parent:
                return self.parent.get( id )
            else:
                raise RuntimeError( 'unknown id %s' % id )
            
        def add( self, type, name, val ):
            self.vars.append( Sharp.Var( type, name, val ) )
            
        def set( self, name, val ):
            var = self.getLocal( name )
            if Sharp.canConvert( val.type, var.type ):
                var.val = val
            else:
                raise RuntimeError( 'trying to set var %s of type %s with unconvertable val %s' % ( name, var.type, val ) )
                
        def has( self, name ):
            return any( [ var.name == name for var in self.vars ] ) or ( self.parent and self.parent.has( name ) )
                
        def canApply( self, name, args ):
            return any( [ var.val.canApply( args ) for var in self.getAll( name ) ] )
            
        def apply( self, name, args ):
            funcs = filter( lambda var: var.val.canApply( args ), self.getAll( name ) )
            if len( funcs ) > 1:
                raise RuntimeError( 'ambigous call %s' % name )
            elif len( funcs ) == 1:
                return funcs[0].apply( args )
            elif self.parent:
                return self.parent.apply( name, args )
            else:
                raise RuntimeError( 'unknown call %s' % name )
            
    @staticmethod
    def canConvert( fromType, toType ):
        return fromType == toType
            
    @staticmethod
    def defaultScope():
        scope = Sharp.Scope( None )
        for name, type in [ ( name, getattr( Sharp.Builtins, name ) ) for name in dir( Sharp.Builtins ) ]:
            scope.add( 
                Sharp.Vals.BuiltinClass.bind( Sharp.Vals.BuiltinClass ),
                name,
                Sharp.Vals.BuiltinClass.bind( type )
            )
        return scope
        
    parser = Parser.build( """
        int = '[-]?\d+';
        str = '".*?"';
        id = '[a-zA-Z_][a-zA-Z0-9_]*';
        ws ~= '\s+';
        program => statement*;
        statement => exprStatement | decl;
        exprStatement => expr ';';
        expr => ref | int | str;
        ref => id ( '\.' id )*;
        decl => ref id ( '=' expr )? ';';
    """ )
    
    @staticmethod
    def eval( input ):
        scope = Sharp.defaultScope()
        return [ Sharp.Exprs.Expr.parse( child ).eval( scope ) for child in Sharp.parser.parse( input ).children ][-1]
    
    @staticmethod
    def test():
        Sharp.eval( """
            False;
            3;
            "foo";
            Bool b = True;
            Int i = 3;
            Str s = "hello";
        """ )
        
if __name__ == '__main__':
    Lexer.test()
    Parser.test()
    Lisp.test()
    Sharpy.test()
    Sharp.test()
