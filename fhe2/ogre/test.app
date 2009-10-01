<type>Window</type>

<children>
    <child>
        <type>Mesh</type>
        <vars>
            <var name="name">robot.mesh</var>
            <var name="scale">Vec3(1,2,1)</var>
        </vars>
    </child>
    <child>
        <type>Button</type>
        <vars>
            <var name="pos">Vec2(0.8,0.9)</var>
            <var name="size">Vec2(0.2,0.1)</var>
            <var name="text">quit</var>
        </vars>
        <scripts>
            <script>quitButton.py</script>
        </scripts>
    </child>
    <child>
        <type>CubeMesh</type>
        <vars>
            <var name="pos">Vec3(-100,0,0)</var>
            <var name="material">Examples/Robot</var>
        </vars>
    </child>
    <child>
        <type>PythonConsole</type>
        <vars>
            <var name="pos">Vec2(0,0)</var>
            <var name="size">Vec2(0.4,0.4)</var>
        </vars>
    </child>
</children>
