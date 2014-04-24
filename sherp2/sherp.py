import re

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
                if result.rule.name in ( 'orMoreArg', 'andOrArg' ):
                    return Parser.Exprs.Expr.parse( result.children[0] )
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
                    ' '.join( [ token.value for token in tokens[pos:pos+10] ] ) )
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
            '+': Lisp.Vals.Builtin( Lisp.Bulitins.add ),
            '-': Lisp.Vals.Builtin( Lisp.Bulitins.sub ),
            '*': Lisp.Vals.Builtin( Lisp.Bulitins.mul ),
            '/': Lisp.Vals.Builtin( Lisp.Bulitins.div ),
            'define': Lisp.Vals.Builtin( Lisp.define ),
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
    
if __name__ == '__main__':
    Lexer.test()
    Parser.test()
    Lisp.test()
