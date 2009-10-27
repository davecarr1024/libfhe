<name>window</name>
<aspects>
    <aspect name="Window"/>
</aspects>
<children>
    <!--child>
        <name>robot</name>
        <aspects>
            <aspect name="Mesh"/>
        </aspects>
        <vars>
            <var name="name">robot.mesh</var>
            <var name="scale">Vec3(1,2,1)</var>
        </vars>
    </child-->
    <child>
        <name>quitButton</name>
        <aspects>
            <aspect name="Button">
                <script>quitButton.py</script>
            </aspect>
        </aspects>
        <vars>
            <var name="pos">Vec2(0.8,0.9)</var>
            <var name="size">Vec2(0.2,0.1)</var>
            <var name="text">quit</var>
        </vars>
    </child>
    <!--child>
        <name>cube</name>
        <aspects>
            <aspect name="CubeMesh"/>
        </aspects>
        <vars>
            <var name="pos">Vec3(-100,0,0)</var>
            <var name="material">Examples/Robot</var>
        </vars>
    </child>
    <child>
        <name>pythonConsole</name>
        <aspects>
            <aspect name="PythonConsole"/>
        </aspects>
        <vars>
            <var name="pos">Vec2(0,0)</var>
            <var name="size">Vec2(0.4,0.4)</var>
        </vars>
    </child-->
</children>
