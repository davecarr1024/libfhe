<aspects>
    <aspect name="Graphics/Window"/>
</aspects>

<vars>
    <!--var name="res" type="vec2">1920 1080</var-->
    <!--var name="fullscreen" type="bool">1</var-->
    <var name="clearColor" type="color">0 0 0 1</var>
</vars>

<children>
    <child name="testFrame">
        <aspects>
            <aspect name="Graphics/Gui/Frame"/>
        </aspects>
        <vars>
            <var name="pos" type="vec2">0.5 0.5</var>
            <var name="scale" type="vec2">0.5 0.5</var>
            <var name="title" type="string">a window</var>
            <var name="fill" type="dict">
                <var name="color" type="color">0 0 1 0.25</var>
            </var>
        </vars>
        <children>
        
            <child name="testSpinner">
                <aspects>
                    <aspect name="Graphics/Gui/Spinner"/>
                </aspects>
                <vars>
                    <var name="pos" type="vec2">0.1 0.25</var>
                    <var name="scale" type="vec2">0.4 0.2</var>
                    <var name="values" type="list">
                        <var type="string">option1</var>
                        <var type="string">option2</var>
                    </var>
                </vars>
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
            
            <child name="testTextBox">
                <aspects>
                    <aspect name="Graphics/Gui/TextBox"/>
                </aspects>
                <vars>
                    <var name="pos" type="vec2">0.1 0.75</var>
                    <var name="scale" type="vec2">0.4 0.2</var>
                    <var name="prompt" type="string">enter text: </var>
                </vars>
            </child>
            
            <child name="testSlider">
                <aspects>
                    <aspect name="Graphics/Gui/Slider"/>
                </aspects>
                <vars>
                    <var name="pos" type="vec2">0.55 0.25</var>
                    <var name="scale" type="vec2">0.4 0.2</var>
                    <var name="prompt" type="string">val:</var>
                </vars>
            </child>
            
            <child name="testRadio">
                <aspects>
                    <aspect name="Graphics/Gui/Radio"/>
                </aspects>
                <vars>
                    <var name="pos" type="vec2">0.55 0.5</var>
                    <var name="scale" type="vec2">0.4 0.1</var>
                    <var name="text" type="string">option1</var>
                </vars>
            </child>
            
            <child name="testRadio">
                <aspects>
                    <aspect name="Graphics/Gui/Radio"/>
                </aspects>
                <vars>
                    <var name="pos" type="vec2">0.55 0.62</var>
                    <var name="scale" type="vec2">0.4 0.1</var>
                    <var name="text" type="string">option2</var>
                </vars>
            </child>
            
        </children>
    </child>
    
    <child name="testCamera">
        <aspects>
            <aspect name="Graphics/Prims/Camera"/>
        </aspects>
        
        <children>
        
            <child name="testCube">
                <aspects>
                    <aspect name="Graphics/Prims/Cube"/>
                </aspects>
                
                <vars>
                    <var name="material" type="dict">
                        <var name="texture" type="string">test.jpg</var>
                    </var>
                    <var name="scale" type="vec3">2 3 4</var>
                    <var name="rot" type="quat">0 1 0 20</var>
                </vars>
                
            </child>
        
        </children>
        
    </child>
    
</children>
