var cmd = {}

var currentGPSOdometry = {x:0, y:0, z:0};

init();

var Configurations = {
    graphs: {
        maxPoints: 600 // 2 minuts @ 5Hz
    }
}

var dataRTK = [];

var plotRTKControl = $.plot($("#PlotRTKControl"), [[]], {
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
    colors: ["#26B99A", "#FE642E", "#808080"],
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

plotRTKControl.getPlaceholder().bind("plotclick", function(event, pos) {
    console.log(event, pos);
    // distance entre les touchés successifs
});

function init() {
    //var cmd = {}
    var ros = new ROSLIB.Ros({
        url: 'ws://192.168.0.101:9090'
    });

    ros.on('connection', function() {
        console.log('Connected to the ROS server.');
            Messenger().post({type:"success", "message":"Connection au serveur ROS réussie"});
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
            Messenger().post({type:"error", "message":"Erreur de connection au serveur ROS"});
    });

    ros.on('close', function() {
        Messenger().post({type:"error", "message":"Connection avec le serveur ROS abandonnée"});
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

cmd.listen.subscribe(function(message){
    plotLocalRTK(message.gps_odometry.pose.pose.position)
    currentGPSOdometry = message.gps_odometry.pose.pose.position
});

function plotLocalRTK(odometry) {
    var dataset = [{
        label: "Position RTK",
        data: [
            [odometry.x, odometry.y]
        ],
        hoverable: false,
        lines: {
          show:true
        }
    }, {
        label: "Base RTK",
        data: [
            [0, 0]
        ],
        hoverable: false,
        points: {
            symbol: "cross"
        }
    }];
    plotRTKControl.setData(dataset);
    plotRTKControl.draw();
}

function plotLocalXYZ(time, point) {
    dataRTK.x
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

plotLocalRTK({x:0, y:0})
