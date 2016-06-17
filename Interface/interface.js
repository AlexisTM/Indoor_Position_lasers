var cmd = {}
var setpoint_x = 0;
init();
var Configurations = {
    ip: 'ws://192.168.43.174:9090',
    setpoints: {
        min: 0.5,
        max: 3.0
    },
    graphs: {
        maxPoints: 50
    }
}


var currentSelection = {
    x: 1,
    y: 1,
    z: 1
}

var clickedSelection = {
    x: 1,
    y: 1,
    z: 1
}


var dataXYZ = {
    x: [
        []
    ],
    y: [
        []
    ],
    z: [
        []
    ]
};


var plotZControl = $.plot($("#PlotLocalZControl"), [[]], {
    yaxis: {
        min: 0,
        max: 3,
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


var plotXY = $.plot($("#PlotLocalXY"), [{
    label: "Local XY",
}], {
    yaxis: {
        min: 0,
        max: 4,
        position: "right",
        reverseSpace: true,
        transform: function(a) {
            return -a;
        },
        inverseTransform: function(a) {
            return -a;
        }
    },
    xaxis: {
        min: 0,
        max: 4,
        position: "top",
        reverseSpace: true,
        transform: function(a) {
            return -a;
        },
        inverseTransform: function(a) {
            return -a;
        }
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
    if (pos.y < Configurations.setpoints.min)
        currentSelection.z = Configurations.setpoints.min
    else if (pos.y > Configurations.setpoints.max)
        currentSelection.z = Configurations.setpoints.max
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

var plotXYZ = $.plot($("#PlotLocalZ"), [{
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
        min: 0,
        max: 2.5
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
        url: 'ws://192.168.137.18:9090'
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

    cmd = {
        Mission: sendMission,
        Task: sendTask,
        CSV: {
            Run: sendCMDCSV,
            Save: sendSaveCSV
        },
        listen: listener,
        listen2: listener2
    };
    //return cmd
}
/*
var ros = new ROSLIB.Ros({
    url: 'ws://192.168.137.18:9090'
});
ros.on('connection', function() {
    console.log('Connected to websocket server.');
});
ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});
ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});*/

// Publishing a Topic
// ------------------

/*var cmdVel = new ROSLIB.Topic({
  ros : ros,
  name : '/cmd_vel',
  messageType : 'geometry_msgs/Twist'
});
var exampleTopic = new ROSLIB.Topic({
      ros: ros,
    name: '/com/endpoint/examp', // use a sensible namespace
      messageType: 'std_msgs/String'
});
var twist = new ROSLIB.Message({
linear : {
  x : 0.1,
  y : 0.2,
  z : 0.3
},
angular : {
  x : -0.1,
  y : -0.2,
  z : -0.3
}
});
var msg = new ROSLIB.Message({
    data : "yesouiok"
});
  console.log("Publishing cmd_vel");
  cmdVel.publish(twist);
  console.log("Publishing data");
  exampleTopic.publish(msg);   
*/


var data = {};

cmd.listen.subscribe(function(message) {
    //console.log(message);
    data = message;

    changePose('#laser_filtered', data.laser_filtered.position)
    changePose('#mocap', data.mocap.position)
    changePose('#setpoint', data.setpoint.position)
    changePose('#laser_pose', data.lasers_pose.position)
    changePose('#local_pose', data.local.position)
    changeRaw('#lasers_raw', data.lasers_raw)

    document.getElementById("pitch").innerHTML = data.local.orientation.pitch.toFixed(3);
    document.getElementById("yaw").innerHTML = data.local.orientation.yaw.toFixed(3);
    document.getElementById("roll").innerHTML = data.local.orientation.roll.toFixed(3);
    document.getElementById("yaw_lasers").innerHTML = data.lasers_pose.orientation.yaw.toFixed(3);
    document.getElementById("yaw_filtered").innerHTML = data.laser_filtered.orientation.yaw.toFixed(3);
    //document.getElementById("battery_current").innerHTML = data.battery.current;
    //document.getElementById("battery_remaining").innerHTML = data.battery.remaining;
    document.getElementById("battery_voltage").innerHTML = data.battery.voltage.toFixed(3);
    document.getElementById("setpoint_yaw").innerHTML = data.setpoint.orientation.yaw.toFixed(3);
    document.getElementById("arm").innerHTML = data.armed;
    document.getElementById("mode").innerHTML = data.guided;

    plotLocalXY(data.local.position.x, data.local.position.y,
        data.setpoint.position.x, data.setpoint.position.y);
    plotLocalZ(data.local.position.z, data.setpoint.position.z),
    plotLocalXYZ(data.header.seq / 25, data.local.position);

    $("div#selector").children().removeClass("btn-primary")
    $("button:contains('" + data.mode + "')").addClass("btn-primary")

});




function changePose(id, position) {
    var elem = $(id);
    elem.find('.datax').text(position.x.toFixed(3));
    elem.find('.datay').text(position.y.toFixed(3));
    elem.find('.dataz').text(position.z.toFixed(3));
}

function showValue(newValue) {
    document.getElementById("range").innerHTML = newValue;
    console.log(newValue);
    var msg = new ROSLIB.Message({
        position: {
            z: newValue
        }
    });
    cmd.Task.publish(msg)
    console.log("Publish Z");

}

function changeRaw(id, raw) {
    var elem = $(id);
    elem.find('.datax1').text(raw.lasers[0]);
    elem.find('.datax2').text(raw.lasers[1]);
    elem.find('.datay1').text(raw.lasers[2]);
    elem.find('.datay2').text(raw.lasers[3]);
    elem.find('.dataz1').text(raw.lasers[4]);
    elem.find('.dataz2').text(raw.lasers[5]);

}

function changeStatus(id, raw) {
    var elem = $(id);
    elem.find('.datax1').text(raw.status[0].toFixed(3));
    elem.find('.datax2').text(raw.status[1].toFixed(3));
    elem.find('.datay1').text(raw.status[2].toFixed(3));
    elem.find('.datay2').text(raw.status[3].toFixed(3));
    elem.find('.dataz1').text(raw.status[4].toFixed(3));
    elem.find('.dataz2').text(raw.status[5].toFixed(3));

}



$("button.landing").click(landing)

function landing() {
    var msg = new ROSLIB.Message({
        mission_type: 123
    });
    cmd.Task.publish(msg)
    console.log("publish landing");
}

$("button.motorstop").click(motorstop)

function motorstop() {
    var msg = new ROSLIB.Message({
        mission_type: 11
    });
    cmd.Task.publish(msg)
    console.log("Publish stop");
}

$("button.actualpose").click(actualpose)

function actualpose() {
    var msg = new ROSLIB.Message({
        position: {
            x: 2.43,
            y: 1.37
        },
        yaw: 0.0
    });
    cmd.Task.publish(msg)
    console.log("Publishing actual");


}


$("div#selector").children().click(function(event) {
    var modeToSend = event.target.innerHTML;
    console.log(modeToSend);
    if (modeToSend == "OFFBOARD") {
        var msg = new ROSLIB.Message({
            mission_type: 9
        });
        cmd.Task.publish(msg)
        console.log("Publish OFFBOARD");
    }

})

$("input.arm").change(function() {
    if (document.getElementById('option1').checked) {
        var msg = new ROSLIB.Message({
            mission_type: 13
        });
        cmd.Task.publish(msg)
        console.log("Publish Arm");
    } else {
        var msg = new ROSLIB.Message({
            mission_type: 11
        });
        cmd.Task.publish(msg)
        console.log("Publish Disarm");


    }
});


function plotLocalXY(x, y, sx, sy) {
    var dataset = [{
        label: "position",
        data: [
            [x, y]
        ],
        hoverable: false,
        points: {
            symbol: "circle",
        }
    }, {
        label: "setpoint",
        data: [
            [sx, sy]
        ],
        hoverable: false,

        points: {
            symbol: "cross"
        }
    }, {
        data: [
            [currentSelection.x, currentSelection.y]
        ],
        hoverable: false,

        points: {
            symbol: "square"
        }
    }];
    plotXY.setData(dataset);
    plotXY.draw();
}

function plotLocalZ(z, sz) {
    var dataset = [{
        label: "position",
        data: [
            [0, z]
        ],
        hoverable: false,
        bars: {
            fill: true,
            show: true
        }
    }, {
        label: "setpoint",
        data: [
            [0.5, sz]
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

function plotLocalXYZ(time, point) {

    dataXYZ.x.push([time, point.x])
    dataXYZ.y.push([time, point.y])
    dataXYZ.z.push([time, point.z])

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
    if (dataXYZ.x.length > Configurations.graphs.maxPoints) {
        dataXYZ.x.shift()
        dataXYZ.y.shift()
        dataXYZ.z.shift()
    }

    plotXYZ.setupGrid();
    plotXYZ.draw();
}


plotLocalXY(1,1,1,1);
plotLocalZ(1.2,1.1);