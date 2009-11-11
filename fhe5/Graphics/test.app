<aspects>
    <aspect name="Graphics/Window"/>
</aspects>

<vars>
    <var name="res" type="vec2">500 500</var>
    <var name="clearColor" type="color">0 0 0 1</var>
</vars>

<children>
    <child name="testFrame">
        <aspects>
            <aspect name="Graphics/Gui/Frame"/>
        </aspects>
        <vars>
            <var name="pos" type="vec2">0.1 0.1</var>
            <var name="scale" type="vec2">0.8 0.8</var>
            <var name="title" type="string">test window</var>
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
                </vars>
            </child>
            
        </children>
    </child>
</children>
