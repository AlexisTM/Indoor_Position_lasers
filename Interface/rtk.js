var cmd = {}

var dataRTK, savedData;
resetAllClick()
init();

var Configurations = {
    graphs: {
        maxPoints: 600 // 2 minuts @ 5Hz
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
        min: -5,
        max: 5,
        position: "center",
        show: true
    },
    xaxis: {
        min: -5,
        max: 5,
        position: "center",
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

    cmd = {
        listen: listener
    };
}

cmd.listen.subscribe(function(message) {
    plotLocalRTK(message.gps_odometry.pose.pose.position)
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



$("<div class='button' style='left:20px;top:20px'>zoom out</div>")
    .appendTo(plotRTKControl.getPlaceholder())
    .click(function(event) {
        event.preventDefault();
        plot.zoomOut();
    });

function addValueClick() {
    savedData.two = savedData.one;
    savedData.one = currentGPSOdometry;
    updateDistance();
}

function updateDistance() {
    $('#data1').val("(" + savedData.one.x + "," + savedData.one.y + ") m");
    $('#data2').val("(" + savedData.two.x + "," + savedData.two.y + ") m");
    $('#distance').val("distance: " + Math.pow(Math.pow((savedData.one.x - savedData.two.x), 2) + Math.pow((savedData.one.y - savedData.two.y), 2), 0.5) + "m");
}

plotLocalRTK({
    x: 0,
    y: 0
})