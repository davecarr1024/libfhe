<aspects>
    <aspect name="Graphics/Window"/>
</aspects>

<vars>
    <var name="res" type="vec2">500 500</var>
    <var name="clearColor" type="color">0 0 0 1</var>
</vars>

<children>
    <child name="rect">
        <aspects>
            <aspect name="Graphics/Prims/Rect"/>
        </aspects>
        <vars>
            <var name="pos" type="vec2">0.1 0.1</var>
            <var name="scale" type="vec2">0.8 0.8</var>
            <var name="material" type="dict">
                <var name="color" type="color">1 0 0 1</var>
                <var name="texture" type="string">test.jpg</var>
            </var>
        </vars>
        <children>
        
            <child name="box">
                <aspects>
                    <aspect name="Graphics/Prims/Rect"/>
                </aspects>
                <vars>
                    <var name="scale" type="vec2">1 0.1</var>
                    <var name="filled" type="bool">0</var>
                    <var name="material" type="dict">
                        <var name="color" type="color">1 1 1 1</var>
                        <var name="texture" type="string"/>
                    </var>
                </vars>
                <children>
                
                    <child name="text">
                        <aspects>
                            <aspect name="Graphics/Prims/Text"/>
                        </aspects>
                        <vars>
                            <var name="text" type="string">hello</var>
                            <var name="material" type="dict">
                                <var name="color" type="color">1 1 1 1</var>
                            </var>
                        </vars>
                    </child>
                    
                </children>
            </child>
            
            <child name="testButton">
                <aspects>
                    <aspect name="Graphics/Gui/Button"/>
                </aspects>
                <vars>
                    <var name="pos" type="vec2">0.1 0.5</var>
                    <var name="scale" type="vec2">0.4 0.2</var>
                    <var name="text" type="string">button!</var>
                </vars>
            </child>
            
        </children>
    </child>
</children>
