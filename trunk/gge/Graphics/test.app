<entity name="Window">
    <aspects>
        <aspect name="Graphics/Window"/>
    </aspects>
</entity>

<entity name="Robot">
    <aspects>
        <aspect name="Graphics/Prims/SceneNode"/>
        <aspect name="Graphics/Prims/Mesh"/>
    </aspects>
    <vars>
        <var name="meshName" type="string">robot.mesh</var>
        <var name="scale" type="Vec3">0.5 1 0.5</var>
        <var name="pos" type="Vec3">50 50 50</var>
    </vars>
</entity>

<entity name="Button">
    <aspects>
        <aspect name="Graphics/Gui/Button"/>
    </aspects>
    <vars>
        <var name="pos" type="Vec2">0.8 0.8</var>
        <var name="size" type="Vec2">0.2 0.2</var>
        <var name="text" type="string">hello</var>
    </vars>
</entity>
