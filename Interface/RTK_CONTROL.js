var Configurations = {
    ip: 'ws://192.168.0.101:9090',
    setpoints: {
        min: -15.0,
        max: 15.0,
        altitude: 15
    },

    graphs: {
        maxPoints: 200
    }
}

var cmd = {}
var altitudes = {
    laser: 0,
    setpoint: 0,
    piksi: 0,
    position: 0
}

var dataXYZ = {
    x: [
        [0, 0]
    ],
    y: [
        [0, 0]
    ],
    z: [
        [0, 0]
    ]
}

var dataXY = {
    position: [
        [0, 0]
    ],
    gps: [
        [0, 0]
    ],
    setpoint: [
        [0, 0]
    ]
};

init();

var currentPosition = {
    x: 0,
    y: 0,
    z: 0
}

var currentSelection = {
    x: 0,
    y: 0,
    z: 0
}

var clickedSelection = {
    x: 0,
    y: 0,
    z: 0
}



var plotZControl = $.plot($("#PlotLocalZControl"), [
    []
], {
    yaxis: {
        min: 0,
        max: 20,
        position: "center"
    },
    xaxis: {
        min: 0,
        max: 0,
        position: "bottom",
        show: false
    },
    bars: {
        fill: true,
        radius: 7,
        show: true
    },
    colors: ["#26B99A", "#FE642E", "#808080"],
    grid: {
        borderWidth: {
            top: 0,
            right: 1,
            bottom: 0,
            left: 1
        },
        borderColor: {
            bottom: "#7F8790",
            left: "#7F8790"
        },
        hoverable: true,
        clickable: true,
        shadowSize: 1
    },
    selection: {
        mode: "xy",
        show: false,
        color: 'rgba(0,0,0,0)'
    }
});

//options.xaxes[0].max = options.yaxes[0].max * $('#ph').width() / $('#ph').height();
//$.plot($('#ph'), data, options);


var plotXY = $.plot($("#PlotLocalXY"), [{
    label: "Local XY",
}], {
    series: {
        curvedLines: {
            apply: true,
            active: true,
            monotonicFit: true
        }
    },
    yaxis: {
        min: -20,
        max: 20,
        position: "center"
    },
    xaxis: {
        min: -20,
        max: 20, // * $("#PlotLocalXY").width() / $("#PlotLocalXY").height() / 2,
        position: "center"
    },
    points: {
        fill: false,
        radius: 7,
        show: true
    },
    colors: ["#26B99A", "#FE642E", "#808080"],
    grid: {
        borderWidth: {
            top: 0,
            right: 0,
            bottom: 1,
            left: 1
        },
        borderColor: {
            bottom: "#7F8790",
            left: "#7F8790"
        },
        hoverable: true,
        clickable: true,
        shadowSize: 1
    },
    selection: {
        mode: "xy",
        show: false,
        color: 'rgba(0,0,0,0)'
    }
});

plotXY.getPlaceholder().bind("plothover", function(event, pos) {
    if (pos.x < Configurations.setpoints.min)
        currentSelection.x = Configurations.setpoints.min
    else if (pos.x > Configurations.setpoints.max)
        currentSelection.x = Configurations.setpoints.max
    else
        currentSelection.x = pos.x

    if (pos.y < Configurations.setpoints.min)
        currentSelection.y = Configurations.setpoints.min
    else if (pos.y > Configurations.setpoints.max)
        currentSelection.y = Configurations.setpoints.max
    else
        currentSelection.y = pos.y
});

plotXY.getPlaceholder().bind("plotclick", function(event, pos) {
    // Send setpoint
    clickedSelection.x = currentSelection.x;
    clickedSelection.y = currentSelection.y;
    console.log("Sending setpoint : ", clickedSelection);
    var msg = new ROSLIB.Message({
        position: {
            x: clickedSelection.x,
            y: clickedSelection.y,
            z: clickedSelection.z
        },
        yaw: 0.0
    });
    cmd.Task.publish(msg)

});

plotZControl.getPlaceholder().bind("plothover", function(event, pos) {
    if (pos.y < 0)
        currentSelection.z = 0
    else if (pos.y > Configurations.setpoints.altitude)
        currentSelection.z = Configurations.setpoints.altitudek
    else
        currentSelection.z = pos.y
});

plotZControl.getPlaceholder().bind("plotclick", function(event, pos) {
    // Send setpoint
    clickedSelection.z = currentSelection.z;
    console.log("Sending setpoint : ", clickedSelection);
    var msg = new ROSLIB.Message({
        position: {
            x: clickedSelection.x,
            y: clickedSelection.y,
            z: clickedSelection.z
        },
        yaw: 0.0
    });
    cmd.Task.publish(msg)

});

var plotXYZ = $.plot($("#PlotLocalXYZ"), [{
    label: "Local position variations",
}], {
    series: {
        curvedLines: {
            apply: true,
            active: true,
            monotonicFit: true
        }
    },
    yaxis: {
        min: -10,
        max: 10
    },
    lines: {
        fillColor: "rgba(150, 0, 89, 0.12)",
        show: true
    }, //#96CA59 rgba(150, 202, 89, 0.42)
    points: {
        //fillColor: "#fff",
        radius: 7,
        show: false,
        symbol: "circle"
    },
    colors: ["#26B99A"],
    grid: {
        borderWidth: {
            top: 0,
            right: 0,
            bottom: 1,
            left: 1
        },
        borderColor: {
            bottom: "#7F8790",
            left: "#7F8790"
        }
    }
});



function init() {
    //var cmd = {}
    var ros = new ROSLIB.Ros({
        //url: 'ws://192.168.43.174:9090'
        url: Configurations.ip
    });

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    });

    var listener = new ROSLIB.Topic({
        ros: ros,
        name: 'web/report',
        messageType: 'laserpack/Report'
    });

    var listener2 = new ROSLIB.Topic({
        ros: ros,
        name: '/new',
        messageType: 'laserpack/Task'
    });

    var sendMission = new ROSLIB.Topic({
        ros: ros,
        name: 'web/mission',
        messageType: 'laserpack/Mission'
    });

    var sendTask = new ROSLIB.Topic({
        ros: ros,
        name: 'web/task',
        messageType: 'laserpack/Task'
    });

    var sendCMDCSV = new ROSLIB.Topic({
        ros: ros,
        name: 'web/csv/run',
        messageType: 'Bool'
    });

    var sendSaveCSV = new ROSLIB.Topic({
        ros: ros,
        name: 'web/csv/save',
        messageType: 'std_msgs/String'
    });

    var odomlitener = new ROSLIB.Topic({
        ros: ros,
        name: 'gps/rtkfix',
        messageType: 'nav_msgs/Odometry'
    })

    cmd = {
        Mission: sendMission,
        Task: sendTask,
        CSV: {
            Run: sendCMDCSV,
            Save: sendSaveCSV
        },
        listen: listener,
        listen2: listener2,
        odometry: odomlitener
    };
}

cmd.listen.subscribe(function(message) {
    dataXY.position = [
        [message.local.position.x, message.local.position.y]
    ];
    dataXY.setpoint = [
        [message.local.setpoint.x, message.local.setpoint.y]
    ];

    addLocalXYZ(message.header.seq / 5, message.local.position);

    $("div#selector").children().removeClass("btn-primary")
    $("button:contains('" + message.mode + "')").addClass("btn-primary")

    altitudes.laser = message.laser.position.z;
    altitudes.setpoint = message.setpoint.position.z;
    altitudes.position = message.local.position.z;

    currentPosition = message.local.position
});

cmd.odometry.subscribe(function(message) {
    altitudes.piksi = message.pose.pose.position.z;
    addLocalRTK(message.pose.pose.position)
});

function addLocalRTK(odometry) {
    dataXY.gps.push([odometry.x, odometry.y])
        //plotRTKControl.setupGrid();
    if (dataXY.gps.length > Configurations.graphs.maxPoints) {
        dataXY.gps.shift()
    }
}

/* PLOT XYZ Temporel */
function addLocalXYZ(time, point) {
    dataXYZ.x.push([time, point.x])
    dataXYZ.y.push([time, point.y])
    dataXYZ.z.push([time, point.z])

    if (dataXYZ.x.length > Configurations.graphs.maxPoints) {
        dataXYZ.x.shift()
        dataXYZ.y.shift()
        dataXYZ.z.shift()
    }
}

$("button.landing").click(landing)

function landing() {
    clickedSelection = currentPosition;
    var msg = new ROSLIB.Message({
        mission_type: 123,
        position: {
            x: clickedSelection.x,
            y: clickedSelection.y,
            z: clickedSelection.z
        },
        yaw: 0.0
    });
    cmd.Task.publish(msg)
    console.log("publish landing");
}

$("button.decollage").click(decollage)

function decollage() {
    clickedSelection = currentPosition;
    var msg = new ROSLIB.Message({
        mission_type: 122,
        position: {
            x: clickedSelection.x,
            y: clickedSelection.y,
            z: 0
        },
        yaw: 0.0
    });
    cmd.Task.publish(msg)
    console.log("publish decollage");
}

$("button.motorstop").click(motorstop)

function motorstop() {
    clickedSelection = currentPosition;
    var msg = new ROSLIB.Message({
        mission_type: 11,
        position: {
            x: clickedSelection.x,
            y: clickedSelection.y,
            z: clickedSelection.z
        },
        yaw: 0.0
    });
    cmd.Task.publish(msg)
    console.log("Publish stop");
}

$("button.actualpose").click(actualpose)

function actualpose() {
    clickedSelection = currentPosition;
    console.log("Selection is now the current position");
}


$("div#selector").children().click(function(event) {
    var modeToSend = event.target.innerHTML;
    console.log(modeToSend);
    if (modeToSend == "OFFBOARD") {
        var msg = new ROSLIB.Message({
            mission_type: 9,
            position: {
                x: clickedSelection.x,
                y: clickedSelection.y,
                z: clickedSelection.z
            },
            yaw: 0.0
        });
        cmd.Task.publish(msg)
        console.log("Publish OFFBOARD");
    }

})

$("input.arm").change(function() {
    if (document.getElementById('option1').checked) {
        clickedSelection = currentPosition;
        var msg = new ROSLIB.Message({
            mission_type: 13,
            position: {
                x: clickedSelection.x,
                y: clickedSelection.y,
                z: clickedSelection.z
            },
            yaw: 0.0
        });
        cmd.Task.publish(msg)
        console.log("Publish Arm");
    } else {
        clickedSelection = currentPosition;
        var msg = new ROSLIB.Message({
            mission_type: 11,
            position: {
                x: clickedSelection.x,
                y: clickedSelection.y,
                z: clickedSelection.z
            },
            yaw: 0.0
        });
        cmd.Task.publish(msg)
        console.log("Publish Disarm");


    }
});

function plotLocalXY(position) {
    var dataset = [{
        label: "Position RTK",
        data: dataXY.gps,
        hoverable: false,
        lines: {
            show: true
        },
        points:{show:false}
    }, {
        label: "Setpoint",
        data: dataXY.setpoint,
        hoverable: false,
        points: {
            show: true,
            radius: 8,
            symbol: "circle"
        }
    }, {
        label: "Base RTK",
        data: [
            [0, 0]
        ],
        hoverable: false,
        points: {
            show: true,
            radius: 8,
            symbol: "cross"
        }
    }];
    plotXY.setData(dataset);
    plotXY.draw();
}

function plotLocalZ() {
    var dataset = [{
        label: "position",
        data: [
            [0, altitudes.position]
        ],
        hoverable: false,
        bars: {
            fill: true,
            show: true
        }
    }, {
        label: "setpoint",
        data: [
            [0.5, altitudes.setpoint]
        ],
        hoverable: false,
        bars: {
            show: false
        },
        points: {
            symbol: function(ctx, x, y, radius, shadow) {
                ctx.moveTo(x - radius * 25, y);
                ctx.lineTo(x + radius * 25, y);
            },
            show: true
        }
    }, {
        label: "Laser",
        data: [
            [0.5, altitudes.laser]
        ],
        hoverable: false,
        bars: {
            show: false
        },
        points: {
            symbol: function(ctx, x, y, radius, shadow) {
                ctx.moveTo(x - radius * 25, y);
                ctx.lineTo(x + radius * 25, y);
            },
            show: true
        }
    }, {
        label: "Piksi",
        data: [
            [0.5, altitudes.piksi]
        ],
        hoverable: false,
        bars: {
            show: false
        },
        points: {
            symbol: function(ctx, x, y, radius, shadow) {
                ctx.moveTo(x - radius * 25, y);
                ctx.lineTo(x + radius * 25, y);
            },
            show: true
        }
    }, {
        data: [
            [0.5, currentSelection.z]
        ],
        hoverable: true,
        clickable: true,
        bars: {
            show: false
        },
        points: {
            symbol: function(ctx, x, y, radius, shadow) {
                ctx.moveTo(x - radius * 25, y);
                ctx.lineTo(x + radius * 25, y);
            },
            show: true
        }
    }];


    plotZControl.setData(dataset);
    plotZControl.draw();
}

function plotLocalXYZ() {
    /* PLOT XYZ Temporel */
    var dataset = [{
        label: "X",
        data: dataXYZ.x,
    }, {
        label: "Y",
        data: dataXYZ.y,
    }, {
        label: "Z",
        data: dataXYZ.z,
    }];
    plotXYZ.setData(dataset);
    plotXYZ.setupGrid();
    plotXYZ.draw();
}


setInterval(function() {
    plotLocalXYZ();
    plotLocalXY();
    plotLocalZ();
}, 500);


var time = 1
function test() {
    randomValue = 10 * Math.random();

    altitudes = {
        laser: randomValue,
        setpoint: randomValue + 1,
        piksi: randomValue + 2,
        position: randomValue + 3
    }

    randomValueX = Math.floor((Math.random() * 30) - 15);
    randomValueY = Math.floor((Math.random() * 30) - 15);
    randomValueZ = Math.floor((Math.random() * 30) - 15);

    position = {
        x: randomValueX,
        y: randomValueY + 1,
        z: randomValueZ + 2
    }
    addLocalRTK(position)
    addLocalXYZ(time, position)
    time +=0.1
    position = {
        x: randomValueX + 2,
        y: randomValueY + 3,
        z: randomValueZ + 5
    }
    addLocalRTK(position)
    addLocalXYZ(time, position)
    time +=0.1
    position = {
        x: randomValueX + 3,
        y: randomValueY + 4,
        z: randomValueZ + 6
    }
    addLocalRTK(position)
    addLocalXYZ(time, position)
    time +=0.1

}
