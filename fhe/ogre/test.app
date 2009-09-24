<type>Window</type>

<children>
    <child>
        <type>SceneNode</type>
    </child>
    <child>
        <type>Widget</type>
    </child>
    <child>
        <type>Mesh</type>
        <vars>
            <var name="name" type="string">robot.mesh</var>
            <var name="rot" type="Quat">0 1 0 90</var>
            <var name="scale" type="Vec3">1 2 1</var>
        </vars>
    </child>
    <child>
        <type>Button</type>
        <vars>
            <var name="pos" type="Vec2">0.8 0.9</var>
            <var name="size" type="Vec2">0.2 0.1</var>
            <var name="text" type="string">quit</var>
        </vars>
        <scripts>
            <script>quitButton.py</script>
        </scripts>
    </child>
    <child>
        <type>CubeMesh</type>
        <vars>
            <var name="pos" type="Vec3">-100 0 0</var>
            <!--var name="scale" type="Vec3">0.1 0.1 0.1</var-->
            <var name="material" type="string">Examples/Robot</var>
        </vars>
    </child>
</children>
