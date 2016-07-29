var cmd = {}

var dataRTK, savedData;
resetAllClick()
init();

var Configurations = {
    graphs: {
        maxPoints: 200 // 2 minuts @ 5Hz
    }
}

var plotRTKControl = $.plot($("#PlotRTKControl"), [
    []
], {
    coordinate: {
        type: "auto",
        ratioXY: 1
    },
    zoom: {
        interactive: true
    },
    pan: {
        interactive: true
    },
    yaxis: {
        min: -30,
        max: 30,
        show: true
    },
    xaxis: {
        min: -30,
        max: 30,
        show: true
    },
    colors: ["#00B18C", "#FEBFD5", "#FE642E"],
    grid: {
        borderWidth: {
            top: 1,
            right: 1,
            bottom: 1,
            left: 1
        },
        borderColor: {
            bottom: "#7F8790",
            left: "#7F8790"
        },
        clickable: true,
        shadowSize: 0
    }
});


function init() {
    //var cmd = {}
    var ros = new ROSLIB.Ros({
        url: 'ws://192.168.0.101:9090'
    });

    ros.on('connection', function() {
        console.log('Connected to the ROS server.');
        Messenger().post({
            type: "success",
            "message": "Connection au serveur ROS réussie"
        });
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        Messenger().post({
            type: "error",
            "message": "Erreur de connection au serveur ROS"
        });
    });

    ros.on('close', function() {
        Messenger().post({
            type: "error",
            "message": "Connection avec le serveur ROS abandonnée"
        });
    });

    var listener = new ROSLIB.Topic({
        ros: ros,
        name: 'web/report',
        messageType: 'laserpack/Report'
    });

    var odomlitener = new ROSLIB.Topic({
          ros: ros,
          name: 'gps/rtkfix',
          messageType: 'nav_msgs/Odometry'
    })

    cmd = {
        listen: listener,
        odometry: odomlitener
    };
}

/*cmd.listen.subscribe(function(message) {
    console.log(message)
    plotLocalRTK(message.gps_odometry.pose.pose.position)
});*/
cmd.odometry.subscribe(function(message) {
    //console.log(message)
    plotLocalRTK(message.pose.pose.position)
});

function plotLocalRTK(odometry) {
    currentGPSOdometry = odometry
    dataRTK.push([odometry.x, odometry.y])
    var dataset = [{
        label: "Position RTK",
        data: dataRTK,
        hoverable: false,
        lines: {
            show: true
        }
    }, {
        label: "Distance meter",
        data: [
            [savedData.one.x, savedData.one.y],
            [savedData.two.x, savedData.two.y]
        ],
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
    plotRTKControl.setData(dataset);
    plotRTKControl.setupGrid();
    plotRTKControl.draw();

    if (dataRTK.length > Configurations.graphs.maxPoints) {
        dataRTK.shift()
    }
}

plotLocalRTK({
    x: 0,
    y: 0
})

function resetAllClick() {
    dataRTK = [];
    savedData = {
        status: 0,
        one: {
            x: 0,
            y: 0
        },
        two: {
            x: 0,
            y: 0
        }
    }
    currentGPSOdometry = {
        x: 0,
        y: 0,
        z: 0
    };
}

Mess

function addValueClick() {
    savedData.two = savedData.one;
    savedData.one = currentGPSOdometry;
    Messenger().post({
        type: "success",
        "message": "Adding point x: " + currentGPSOdometry.x.toFixed(2) + " y: " + currentGPSOdometry.y.toFixed(2) + " value"
    });
    updateDistance();
}

function updateDistance() {
    $('#data1').val("(" + savedData.one.x.toFixed(2) + "," + savedData.one.y.toFixed(2) + ") m");
    $('#data2').val("(" + savedData.two.x.toFixed(2) + "," + savedData.two.y.toFixed(2) + ") m");
    $('#distance').val("distance: " + Math.pow(Math.pow((savedData.one.x - savedData.two.x), 2) + Math.pow((savedData.one.y - savedData.two.y), 2), 0.5).toFixed(2) + "m");
}

plotLocalRTK({
    x: 0,
    y: 0
})
