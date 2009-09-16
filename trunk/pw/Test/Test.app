name: TestApp

children:
    window:
        type: Text.Window

        children:
            menu:
                type: Text.Gui.Menu
                vars:
                    pos: 10,10
                    options: [these,are,some,options]
                    label: hi
                    
            input:
                type: Text.Gui.TextInput
                vars:
                    pos: 5,5
                    prompt: "input:"
