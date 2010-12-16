node = Node( "TestNode" )
node.i = 15
assert node.i == 15

node.set( 1 )
assert node.get() == 1

child = Node( "TestNode" )

assert not node.hasChild( child )
assert not child.parent()
child.attachToParent( node )
assert node.hasChild( child )
assert node == child.parent()
child.detachFromParent()
assert not child.parent()

assert not node.hasChild( child )
assert not child.parent()
node.attachChild( child )
assert node.hasChild( child )
assert node == child.parent()
node.detachChild( child )
assert not node.hasChild( child )
assert not child.parent()

node.attachChild( child )
child.attachChild( Node( "core/Node" ) )
assert node == child.children()[0].root()

