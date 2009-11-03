<entity name="Window">
    <aspects>
        <aspect name="Window"/>
    </aspects>
</entity>

<entity name="World">
    <aspects>
        <aspect name="World"/>
    </aspects>
</entity>

<entity name="Cube">
    <aspects>
        <aspect name="SceneNode"/>
        <aspect name="CubeBody"/>
        <aspect name="Mesh"/>
    </aspects>
    <vars>
        <var name="meshName">robot.mesh</var>
        <var name="pos">Vec3(0,100,0)</var>
    </vars>
</entity>

<entity name="Floor">
    <aspects>
        <aspect name="PlaneBody"/>
    </aspects>
    <vars>
        <var name="mass">0.0</var>
    </vars>
</entity>
