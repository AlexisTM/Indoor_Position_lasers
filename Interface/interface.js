var cmd = {}
init();
var Configurations = {
    setpoints: {
        min: 0.5,
        max: 3.0
    }
}


var currentSelection = {
    x: 1,
    y: 1
}


var dataZ = [
    []
];

var plotXY = $.plot($("#PlotLocalXY"), [{
    label: "Local XY",
}], {
    yaxis: {
        min: 0,
        max: 4
    },
    xaxis: {
        min: 0,
        max: 4
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
        //console.log(currentSelection);
});

plotXY.getPlaceholder().bind("plotclick", function(event, pos) {
    // Send setpoint
    console.log("Sending setpoint : ", currentSelection);
});


var plotZ = $.plot($("#PlotLocalZ"), [{
    label: "Local Z",
    data: dataZ
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



function init()
{
    //var cmd = {}
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
    });

    var listener = new ROSLIB.Topic({
    ros: ros,
    name: 'web/report',
    messageType: 'laserpack/Report'
    });

    var sendMission = new ROSLIB.Topic({
      ros : ros,
      name : 'web/mission',
      messageType : 'laserpack/Mission'
    });

    var sendTask = new ROSLIB.Topic({
      ros : ros,
      name : 'web/task',
      messageType : 'laserpack/Task'
    });

    var sendCMDCSV = new ROSLIB.Topic({
      ros : ros,
      name : 'web/csv/run',
      messageType : 'Bool'
    });

    var sendSaveCSV = new ROSLIB.Topic({
      ros : ros,
      name : 'web/csv/save',
      messageType : 'std_msgs/String'
    });

    cmd = { 
        Mission : sendMission,
        Task : sendTask,
        CSV : {
          Run : sendCMDCSV,
          Save : sendSaveCSV
         },
         listen : listener
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
    plotLocalZ(data.header.seq / 25, data.local.position.z)

    $("div#selector").children().removeClass("btn-primary")
    $("button:contains('" + data.mode + "')").addClass("btn-primary")

});




function changePose(id, position) {
    var elem = $(id);
    elem.find('.datax').text(position.x.toFixed(3));
    elem.find('.datay').text(position.y.toFixed(3));
    elem.find('.dataz').text(position.z.toFixed(3));
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
}

$("button.motorstop").click(motorstop)

function motorstop() {
    alert("motorstop");
}

$("button.actualpose").click(actualpose)

function actualpose() {
    alert("actualpose");

}


$("div#selector").children().click(function(event) {
    var modeToSend = event.target.innerHTML;
    console.log(modeToSend);
})

$("input.arm").change(function() {
    if (document.getElementById('option1').checked) {

        cmd.Task.publish({mission_type : 13})
    } else {
        cmd.Task.publish({mission_type : 11})

    }
});


function plotLocalXY(x, y, sx, sy) {
    var dataset = [{
        label: "position",
        data: [
            [x, y]
        ],
hoverable : false,
        points: {
            symbol: "circle",
        }
    }, {
        label: "setpoint",
        data: [
            [sx, sy]
        ],hoverable : false,

        points: {
            symbol: "cross"
        }
    }, {
        data: [
            [currentSelection.x, currentSelection.y]
        ],hoverable : false,

        points: {
            symbol: "square"
        }
    }];
    plotXY.setData(dataset);
    plotXY.draw();
}


function plotLocalZ(time, z) {
    dataZ.push([time, z])
    plotZ.setData([dataZ]);
    if (dataZ.length > 100)
        dataZ.shift()
    plotZ.setupGrid();
    plotZ.draw();
}