import QtQuick 2.0
import QtQuick.XmlListModel 2.0

Item {
    id: container
    width: 100
    height: 62

    signal clicked(string url)

    Text {
        id: header
        text: qsTr("Project Activity")
        width: parent.width - 32
        color: "#44515c"
        font {
            pointSize: 14
            weight: Font.Bold
        }
    }

    Image {
        id: refresh
        width: 16
        height: 16
        source: "images/refresh.png"
        anchors.left: header.right
        anchors.leftMargin: 6
        anchors.verticalCenter: header.verticalCenter
        MouseArea {
            id: mouseAreaRefresh
            anchors.fill: parent
            hoverEnabled: true
            onClicked: {
                xmlModel.reload()
            }
        }

    }

    ListView {
        id: view
        width: parent.width
        spacing: 8
        anchors { top: header.bottom; topMargin: 14; bottom: parent.bottom }
        model: xmlModel
        delegate: listDelegate
        clip: true
    }

    ScrollDecorator {
        flickableItem: view
    }

    XmlListModel {
        id: xmlModel
        source: ""
        query: ""
        namespaceDeclarations: ""

        XmlRole { }
    }

    Component {
        id: listDelegate
        Item {
            id: item
            width: view.width
            height: column.height
            Column {
                id: column
                Row {
                    id: topRow
                    spacing: 8
                    Image {
                        id: photo
                        width: 16
                        height: 16
                        source: authorPhoto
                        MouseArea {
                            id: mouseArea
                            anchors.fill: parent
                            hoverEnabled: true
                            onClicked: {
                                container.clicked(authorLink)
                            }
                        }
                    }
                    Text {
                        id: name
                        text: {
                            if (author != "") {
                                author
                            } else if (authorEmail != "") {
                                authorEmail
                            } else {
                                "Unknown Author"
                            }
                        }
                        width: container.width - photo.width - icon.width - 24
                        color: mouseArea2.containsMouse ? "#224d81" : "black"
                        wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignTop
                        textFormat: Text.RichText
                        elide: Text.ElideRight
                        font.bold: true
                        MouseArea {
                            id: mouseArea2
                            anchors.fill: parent
                            hoverEnabled: true
                            onClicked: {
                                container.clicked(authorLink)
                            }
                        }

                    }
                    Image {
                        id: icon
                        width: 16
                        height: 16
                        // No image for GitHub commits.
                        // source: applicationIcon
                    }
                }
                Row {
                    id: middleRow
                    anchors.left: parent.left
                    anchors.leftMargin: 25

                    Text {
                        property string prefix: ""
                        width: container.width - anchors.leftMargin - icon.width - 24 - 8
                        wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                        textFormat: Text.RichText
                        elide: Text.ElideRight
                        text: {
                            switch(action) {
                            case "commented":
                            case "comment": prefix = qsTr("Commented on "); break;
                            case "post":
                            case "created": prefix = qsTr("Created "); break;
                            case "create-and-start": prefix = qsTr("Created and started "); break;
                            case "complete": prefix = qsTr("Completed "); break;
                            case "close":
                            case "closed": prefix = qsTr("Closed "); break;
                            case "abandon": prefix = qsTr("Abandoned "); break;
                            case "commit": prefix = qsTr("Committed "); break;
                            case "Commit": prefix = qsTr("Committed "); break;
                            case "resolved": prefix = qsTr("Resolved "); break;
                            case "start": prefix = qsTr("Started "); break;
                            case "started": prefix = qsTr("Started working on "); break;
                            case "stopped": prefix = qsTr("Stopped working on "); break;
                            case "Code Review": prefix = qsTr("Requested code review on "); break;
                            case "Testing": prefix = qsTr("Requested testing of "); break;
                            case "": prefix = qsTr("Updated "); break;
                            default: prefix = action.substr(0, 1).toUpperCase() + action.substr(1) + " " ; break;
                            }
                            prefix = "<font color='#224d81'>" + prefix + "</font>"
                            prefix + actionTitle
                        }
                        color: mouseArea3.containsMouse ? "#224d81" : "black"
                        MouseArea {
                            id: mouseArea3
                            anchors.fill: parent
                            hoverEnabled: true
                            onClicked: {
                                container.clicked(actionLink)
                            }
                        }
                    }
                }
            }
        }
    }
}
