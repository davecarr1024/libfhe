<entity name="Window">
    <aspects>
        <aspect name="Graphics/Window"/>
    </aspects>
</entity>

<entity name="World">
    <aspects>
        <aspect name="Physics/World"/>
    </aspects>
</entity>

<entity name="Cube">
    <aspects>
        <aspect name="Graphics/Prims/SceneNode"/>
        <aspect name="Physics/Cube"/>
        <aspect name="Graphics/Prims/Mesh"/>
    </aspects>
    <vars>
        <var name="meshName">robot.mesh</var>
        <var name="pos">Vec3(0,100,0)</var>
    </vars>
</entity>

<entity name="Floor">
    <aspects>
        <aspect name="Physics/Plane"/>
    </aspects>
    <vars>
        <var name="mass">0.0</var>
    </vars>
</entity>
