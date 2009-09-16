type: Text.Window

children:
    text:
        type: Text.Label
        vars:
            text: hi
            pos: 10,10
            
    box:
        type: Text.Box
        vars:
            pos: 20,20
            size: 10,5
            title: title
            
    yesButton:
        type: Text.Button
        vars:
            pos: 30, 5
            text: "yes"
            
    noButton:
        type: Text.Button
        vars:
            pos: 40, 5
            text: "no"
            
    okButton:
        type: Text.Button
        vars:
            pos: 30, 7
            text: "ok"
            
    cancelButton:
        type: Text.Button
        vars:
            pos: 40, 7
            text: "cancel"
            
    input:
        type: Text.Input
        vars:
            pos: 30, 9
            prompt: "input:"
            
    spinner:
        type: Text.Spinner
        vars:
            pos: 30,11
            options: range(10)
