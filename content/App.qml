// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only
//
// 6-DOF Robotic Arm - Top-level application window
// Author: Luca Obwegs

import QtQuick 6.5
import Robot_GUI2

Window {
    width: mainScreen.width
    height: mainScreen.height

    visible: true
    title: "Robot_GUI2"

    Screen01 {
        id: mainScreen
    }

}

