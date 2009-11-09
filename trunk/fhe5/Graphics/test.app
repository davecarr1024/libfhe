<aspects>
    <aspect name="Graphics/Window"/>
</aspects>

<vars>
    <var name="screenw" type="int">100</var>
    <var name="screenh" type="int">100</var>
</vars>

<children>
    <child name="rect">
        <aspects>
            <aspect name="Graphics/Prims/Rect"/>
        </aspects>
        <vars>
            <var name="pos" type="vec2">0.1 0.1</var>
            <var name="scale" type="vec2">0.1 0.1</var>
            <var name="material" type="dict">
                <var name="color" type="color">0 0 1 1</var>
            </var>
        </vars>
    </child>
</children>
