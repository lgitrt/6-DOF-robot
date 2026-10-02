

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.

6-DOF Robotic Arm - Main teleoperation / teach-in screen layout
Author: Luca Obwegs
*/
import QtQuick 6.5
import QtQuick.Controls 6.5
import Robot_GUI2

Rectangle {
    id: root
    width: Constants.width
    height: Constants.height
    color: "#000000"

    Frame {
        id: rviz_gui
        x: 960
        y: 0
        width: 960
        height: 1080
    }

    Text {
        id: status_box_text
        x: 973
        y: 1047
        width: 679
        height: 33
        color: "#ffffff"
        text: qsTr("Robot not connected!")
        font.pixelSize: 15
    }

    Frame {
        id: js_control_frame
        x: 0
        y: 120
        width: 960
        height: 279
        Label {
            id: label_frame_JS_control
            x: 342
            y: -6
            width: 253
            height: 37
            text: qsTr("Joint Space Control")
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 18
            font.bold: true
        }

        TextField {
            id: j4_textfield
            x: 147
            y: 128
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        TextField {
            id: j1_textfield
            x: 147
            y: 41
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        Slider {
            id: j1_slider
            x: 76
            y: 79
            height: 30
            value: 0.5
        }

        Slider {
            id: j2_slider
            x: 393
            y: 79
            height: 30
            value: 0.5
        }

        Slider {
            id: j3_slider
            x: 713
            y: 79
            height: 30
            value: 0.5
        }

        Slider {
            id: j4_slider
            x: 76
            y: 166
            height: 30
            value: 0.5
        }

        Slider {
            id: j5_slider
            x: 393
            y: 166
            height: 30
            value: 0.5
        }

        Slider {
            id: j6_slider
            x: 713
            y: 166
            height: 30
            value: 0.5
        }

        Label {
            id: j1_label
            x: 6
            y: 79
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("Joint 1")
            font.pointSize: 13
        }

        Label {
            id: j2_label
            x: 317
            y: 79
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("Joint 2")
            font.pointSize: 13
        }

        Label {
            id: j3_label
            x: 632
            y: 79
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("Joint 3")
            font.pointSize: 13
        }

        Label {
            id: j4_label
            x: 6
            y: 166
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("Joint 4")
            font.pointSize: 13
        }

        Label {
            id: j5_label
            x: 317
            y: 166
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("Joint 5")
            font.pointSize: 13
        }

        Label {
            id: j6_label
            x: 632
            y: 166
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("Joint 6")
            font.pointSize: 13
        }

        RoundButton {
            id: set_button
            x: 829
            y: 213
            width: 84
            height: 36
            radius: 9
            text: "Set"
            icon.color: "#de2424"
        }

        RoundButton {
            id: reset_angles
            x: 713
            y: 213
            width: 84
            height: 36
            radius: 9
            text: "Reset"
            icon.color: "#de2424"
        }

        TextField {
            id: j2_textfield
            x: 463
            y: 41
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        TextField {
            id: j5_textfield
            x: 463
            y: 128
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        TextField {
            id: j3_textfield
            x: 783
            y: 41
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        TextField {
            id: j6_textfield
            x: 783
            y: 128
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        Switch {
            id: switch_js_control
            x: 0
            y: 1
            text: qsTr("On/Off")
        }
    }

    Frame {
        id: com_frame
        x: 0
        y: 0
        width: 960
        height: 120
        Label {
            id: label_frame_COM
            x: 342
            y: -12
            width: 253
            height: 37
            text: qsTr("Serial Communication")
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 18
            font.bold: true
        }

        TextField {
            id: textField
            x: 166
            y: 46
            width: 191
            height: 50
            placeholderText: qsTr("/dev/ttyS7")
            font.pointSize: 14
        }

        Label {
            id: label
            x: 16
            y: 52
            width: 144
            height: 38
            text: qsTr("Serial Port:")
            font.pointSize: 16
        }

        RoundButton {
            id: connect_button
            x: 393
            y: 46
            width: 152
            height: 50
            radius: 9
            text: "Connect"
            icon.color: "#de2424"
        }

        Rectangle {
            id: rectangle1
            x: 810
            y: 51
            width: 40
            height: 40
            color: "#527d59"
            radius: 20
            border.color: "#525252"
            border.width: 3
        }

        Rectangle {
            id: rectangle2
            x: 886
            y: 51
            width: 40
            height: 40
            color: "#784d4d"
            radius: 20
            border.color: "#525252"
            border.width: 3
        }
    }

    Frame {
        id: ts_control_frame
        x: 0
        y: 399
        width: 960
        height: 279
        Label {
            id: label_frame_TS_control
            x: 342
            y: -6
            width: 253
            height: 37
            text: qsTr("Task Space Control")
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 18
            font.bold: true
        }

        TextField {
            id: phi_textfield
            x: 147
            y: 128
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        TextField {
            id: x_textfield
            x: 147
            y: 41
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        Slider {
            id: x_slider
            x: 76
            y: 79
            height: 30
            value: 0.5
        }

        Slider {
            id: y_slider
            x: 393
            y: 79
            height: 30
            value: 0.5
        }

        Slider {
            id: z_slider
            x: 713
            y: 79
            height: 30
            value: 0.5
        }

        Slider {
            id: phi_slider
            x: 76
            y: 166
            height: 30
            value: 0.5
        }

        Slider {
            id: theta_slider
            x: 393
            y: 166
            height: 30
            value: 0.5
        }

        Slider {
            id: psi_slider
            x: 713
            y: 166
            height: 30
            value: 0.5
        }

        Label {
            id: x_label
            x: -4
            y: 79
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("X")
            horizontalAlignment: Text.AlignRight
            font.pointSize: 13
        }

        Label {
            id: y_label
            x: 317
            y: 79
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("Y")
            horizontalAlignment: Text.AlignRight
            font.pointSize: 13
        }

        Label {
            id: z_label
            x: 632
            y: 79
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("Z")
            horizontalAlignment: Text.AlignRight
            font.pointSize: 13
        }

        Label {
            id: phi_label
            x: -4
            y: 166
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("PHI")
            horizontalAlignment: Text.AlignRight
            font.pointSize: 13
        }

        Label {
            id: theta_label
            x: 317
            y: 166
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("THETA")
            horizontalAlignment: Text.AlignRight
            font.pointSize: 13
        }

        Label {
            id: psi_label
            x: 632
            y: 166
            width: 70
            height: 30
            color: "#ffffff"
            text: qsTr("PSI")
            horizontalAlignment: Text.AlignRight
            font.pointSize: 13
        }

        RoundButton {
            id: home_button
            x: 829
            y: 212
            width: 84
            height: 36
            radius: 9
            text: "Home"
            icon.color: "#de2424"
        }

        TextField {
            id: y_textfield
            x: 463
            y: 41
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        TextField {
            id: theta_textfield
            x: 463
            y: 128
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        TextField {
            id: z_textfield
            x: 783
            y: 41
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        TextField {
            id: psi_textfield
            x: 783
            y: 128
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("0.0")
        }

        Switch {
            id: switch_ts_control
            x: 0
            y: 1
            text: qsTr("On/Off")
        }

        Label {
            id: label1
            x: -4
            y: 235
            text: qsTr("This function automatically sets the values!")
        }
    }

    Frame {
        id: variables_frame
        x: 0
        y: 678
        width: 960
        height: 402
        RoundButton {
            id: stop_button
            x: 754
            y: 257
            width: 169
            height: 100
            radius: 30
            text: "STOP Robot"
            highlighted: false
            font.bold: true
            flat: false
        }

        TextField {
            id: frequency_textfield
            x: 188
            y: 148
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: loop_frequency_label
            x: 26
            y: 148
            width: 156
            height: 32
            text: qsTr("Loop Frequency:")
            font.pointSize: 12
        }

        Label {
            id: label_frame_variables
            x: 353
            y: -6
            width: 253
            height: 37
            text: qsTr("Variables")
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 18
            font.bold: true
        }

        RoundButton {
            id: reset_everything_button
            x: 754
            y: 148
            width: 169
            height: 100
            radius: 30
            text: "Reset Everything"
            highlighted: false
            font.bold: true
            flat: false
        }

        TextField {
            id: ik_iterations_textfield
            x: 188
            y: 193
            text: "3"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: ik_iterations_label
            x: 26
            y: 193
            width: 156
            height: 32
            text: qsTr("IK Iterations:")
            font.pointSize: 12
        }

        TextField {
            id: frequency_textfield2
            x: 188
            y: 237
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: loop_frequency_label2
            x: 26
            y: 237
            width: 156
            height: 32
            text: qsTr("Spare variable:")
            font.pointSize: 12
        }

        TextField {
            id: frequency_textfield3
            x: 188
            y: 282
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: loop_frequency_label3
            x: 26
            y: 282
            width: 156
            height: 32
            text: qsTr("Spare variable:")
            font.pointSize: 12
        }

        TextField {
            id: frequency_textfield4
            x: 188
            y: 325
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: loop_frequency_label4
            x: 26
            y: 325
            width: 156
            height: 32
            text: qsTr("Spare variable:")
            font.pointSize: 12
        }

        TextField {
            id: frequency_textfield5
            x: 470
            y: 148
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: loop_frequency_label5
            x: 308
            y: 148
            width: 156
            height: 32
            text: qsTr("Spare variable:")
            font.pointSize: 12
        }

        TextField {
            id: frequency_textfield6
            x: 470
            y: 193
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: loop_frequency_label6
            x: 308
            y: 193
            width: 156
            height: 32
            text: qsTr("Spare variable:")
            font.pointSize: 12
        }

        TextField {
            id: frequency_textfield7
            x: 470
            y: 237
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: loop_frequency_label7
            x: 308
            y: 237
            width: 156
            height: 32
            text: qsTr("Spare variable:")
            font.pointSize: 12
        }

        TextField {
            id: frequency_textfield8
            x: 470
            y: 282
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        Label {
            id: loop_frequency_label8
            x: 308
            y: 282
            width: 156
            height: 32
            text: qsTr("Spare variable:")
            font.pointSize: 12
        }

        TextField {
            id: frequency_textfield9
            x: 470
            y: 325
            text: "10.0"
            horizontalAlignment: Text.AlignHCenter
            placeholderText: qsTr("Text Field")
        }

        RoundButton {
            id: set_variables_button
            x: 560
            y: 148
            width: 169
            height: 209
            radius: 40
            text: "Set Variables"
            highlighted: false
            font.bold: true
            flat: false
        }

        Label {
            id: loop_frequency_label9
            x: 308
            y: 325
            width: 156
            height: 32
            text: qsTr("Spare variable:")
            font.pointSize: 12
        }

        Label {
            id: j1_label_variables
            x: 26
            y: 42
            width: 73
            height: 32
            text: qsTr("J1:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: j1_variables_value
            x: 68
            y: 42
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: j2_label_variables
            x: 188
            y: 42
            width: 73
            height: 32
            text: qsTr("J2:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: j2_variables_value
            x: 230
            y: 42
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: j3_label_variables
            x: 341
            y: 42
            width: 73
            height: 32
            text: qsTr("J3:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: j3_variables_value
            x: 383
            y: 42
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: j4_label_variables
            x: 500
            y: 42
            width: 73
            height: 32
            text: qsTr("J4:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: j4_variables_value
            x: 542
            y: 42
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: j5_label_variables
            x: 664
            y: 42
            width: 73
            height: 32
            text: qsTr("J5:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: j5_variables_value
            x: 706
            y: 42
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: j6_label_variables
            x: 833
            y: 42
            width: 73
            height: 32
            text: qsTr("J6:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: j6_variables_value
            x: 875
            y: 42
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: x_label_variables
            x: 26
            y: 87
            width: 73
            height: 32
            text: qsTr("X:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: x_variables_value
            x: 68
            y: 87
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: y_label_variables
            x: 188
            y: 87
            width: 73
            height: 32
            text: qsTr("Y:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: y_variables_value
            x: 230
            y: 87
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: z_label_variables
            x: 341
            y: 87
            width: 73
            height: 32
            text: qsTr("Z:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: z_variables_value
            x: 383
            y: 87
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: phi_label_variables
            x: 500
            y: 87
            width: 73
            height: 32
            text: qsTr("PHI:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: phi_variables_value
            x: 542
            y: 87
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: theta_label_variables
            x: 637
            y: 87
            width: 100
            height: 32
            text: qsTr("THETA:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: theta_variables_value
            x: 706
            y: 87
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }

        Label {
            id: psi_label_variables
            x: 833
            y: 87
            width: 73
            height: 32
            text: qsTr("PSI:")
            font.pointSize: 12
            font.bold: true
        }

        Label {
            id: psi_variables_value
            x: 875
            y: 87
            width: 73
            height: 32
            text: qsTr("0.0")
            font.pointSize: 12
        }
    }
}

/*##^##
Designer {
    D{i:0}D{i:4;locked:true}
}
##^##*/
