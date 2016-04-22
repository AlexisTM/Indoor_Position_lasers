var dataXY = [[]];
var dataZ = [[]];

//var optionsXY = ;
var plotXY = $.plot($("#PlotLocalXY"), [{
      label: "Local XY",
      data: dataXY
    }],
    {
      series: {
        curvedLines: {
          apply: true,
          active: true,
          monotonicFit: true
                  }
      },
      yaxis: {
        min: 0,
        max: 5
      },
      xaxis: {
        min: 0,
        max: 5
      },
      lines: {
        fillColor: "rgba(150, 0, 89, 0.12)",
        show: true
      }, //#96CA59 rgba(150, 202, 89, 0.42)
      points: {
        //fillColor: "#fff",
        radius: 7,
        show: true,
        symbol: "cross"
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
    }
 );

var plotZ = $.plot($("#PlotLocalZ"), [{
      label: "Local Z",
      data: dataZ
    }],
    {
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
    }
 );
/*
 var plot = $.plot($("#PlotLocalZ"), [{
      label: "Local Z",
      data: dataXY,
      lines: {
        fillColor: "rgba(150, 202, 89, 0.12)",
        show: true
      }, //#96CA59 rgba(150, 202, 89, 0.42)
      points: {
        fillColor: "#fff",
        show: true
      }
    }],
    options
  );*/


var ros = new ROSLIB.Ros({
    url : 'ws://192.168.137.18:9090'
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

var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/com/endpoint/examp',
    messageType : 'std_msgs/String'
  });

  listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
    listener.unsubscribe();
  });

var msg = new ROSLIB.Message({
    data : "yesouiok"
});


  console.log("Publishing cmd_vel");
  cmdVel.publish(twist);

  console.log("Publishing data");
  exampleTopic.publish(msg);   
/*
var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/Ack',
  messageType : 'geometry_msgs/PoseStamped'
});
*/
var listener = new ROSLIB.Topic({
  ros : ros,
  name : 'web/report',
  messageType : 'laserpack/Report'
});


var data = {
};

listener.subscribe(function(message) {
    //console.log(message);
    //document.getElementById("test").innerHTML = message.pose.position.y;
   //console.log('Received message on ' + listener.name + ': ' + message.pose.position.y);
    //console.log('Received message on ' + listener.name + ': ' + message.pose.position.z);
  //data.pose = message.pose.position;
  data = message;   
   
  /*changePose("#localpose",{ x:data.local.pose.position.x , y:data.local.pose.position.y, z: data.local.position.pose.z})
  changePose('#mocap',{x:data.mocap.position.x,y:data.mocap.position.y,z:data.mocap.position.z })
  changePose('#setpoint',{x:data.setpoint.position.x,y:data.setpoint.position.y,z:data.setpoint.position.z })
  changePose('#laser_pose',{x:data.laser_pose.position.x,y:data.laser_pose.position.y,z:data.laser_pose.position.z })
  changePose('#laser_filtered',{x:data.laser_filtered.position.x,y:data.laser_filtered.position.y,z:data.laser_filtered.position.z })
*/
  changePose('#laser_filtered', data.laser_filtered.position)
  changePose('#mocap', data.mocap.position)
  changePose('#setpoint', data.setpoint.position)
  changePose('#laser_pose', data.lasers_pose.position)
  changePose('#local_pose', data.local.position)
  changeRaw('#lasers_raw', data.lasers_raw)

  document.getElementById("pitch").innerHTML = data.local.orientation.pitch;
  document.getElementById("yaw").innerHTML = data.local.orientation.yaw;
  document.getElementById("roll").innerHTML = data.local.orientation.roll;
  document.getElementById("yaw_lasers").innerHTML = data.lasers_pose.orientation.yaw;
  document.getElementById("yaw_filtered").innerHTML = data.laser_filtered.orientation.yaw;
  //document.getElementById("battery_current").innerHTML = data.battery.current;
  //document.getElementById("battery_remaining").innerHTML = data.battery.remaining;
  document.getElementById("battery_voltage").innerHTML = data.battery.voltage;
  document.getElementById("setpoint_yaw").innerHTML = data.setpoint.orientation.yaw;
  document.getElementById("arm").innerHTML = data.armed;
  document.getElementById("mode").innerHTML = data.guided;

 data1=data.local.position.x;
 data2=data.local.position.y;
 /*var dataXY = [
      [data1, data2],
    ];*/

plotLocalZ(data.header.seq/25, data.local.position.z)
plotLocalXY(data1,data2);


 });



   
function changePose(id, position){
	var elem = $(id);
	elem.find('.datax').text(position.x);
	elem.find('.datay').text(position.y);
	elem.find('.dataz').text(position.z);
}

function changeRaw(id, raw){
  var elem = $(id);
  elem.find('.datax1').text(raw.lasers[0]);
  elem.find('.datax2').text(raw.lasers[1]);
  elem.find('.datay1').text(raw.lasers[2]);
  elem.find('.datay2').text(raw.lasers[3]);
  elem.find('.dataz1').text(raw.lasers[4]);
  elem.find('.dataz2').text(raw.lasers[5]);

}

function changeStatus(id, raw){
  var elem = $(id);
  elem.find('.datax1').text(raw.status[0]);
  elem.find('.datax2').text(raw.lasers[1]);
  elem.find('.datay1').text(raw.lasers[2]);
  elem.find('.datay2').text(raw.lasers[3]);
  elem.find('.dataz1').text(raw.lasers[4]);
  elem.find('.dataz2').text(raw.lasers[5]);

}


function plotLocalXY(x,y){
  plotXY.setData([[[x,y]]]);
  plotXY.draw();

}

function plotLocalZ(time, z){
  dataZ.push([time, z])
  plotZ.setData([dataZ]);
  if(dataZ.length > 100)
    dataZ.shift()
  plotZ.setupGrid();
  plotZ.draw();
}


 /*changePose("#localpose",{ x:1 , y:3, z: 6});
 changePose("#mocap",{ x:8 , y:8, z: 8});
 changePose("#setpoint",{ x:9 , y:9, z: 9});
 changePose("#laserpose",{ x:1020 , y:1020, z: 1020});
 changePose("#laserfiltered",{ x:12.2 , y:12.2, z: 12.2});
 document.getElementById("yaw_laser").innerHTML = 3;
 document.getElementById("yaw_filter").innerHTML = 6;*/





    //random data for plot 
    /*var dataXY = [
      [0, 1],
      [1, 9],
      [2, 6],
      [3, 10],
      [4, 5],
      [5, 17],
      [6, 6],
      [7, 10],
      [8, 7],
      [9, 11],
      [10, 35],
      [11, 9],
      [12, 12],
      [13, 5],
      [14, 3],
      [15, 4],
      [16, 9]
    ];*/
    
    var d2 = [
      [0, 1],
      [1, 2],
      [2, 6],
      [3, 5],
      [4, 7],
      [5, 17],
      [6, 28],
      [7, 1],
      [8, 7],
      [9, 9],
      [10, 25],
      [11, 9],
      [12, 22],
      [13, 3],
      [14, 3],
      [15, 4],
      [16, 6]
    ];

    var d3 = [
      [0, 12],
      [1, 2],
      [2, 62],
      [3, 100],
      [4, 7],
      [5, 17],
      [6, 28],
      [7, 12],
      [8, 78],
      [9, 9],
      [10, 25],
      [11, 91],
      [12, 22],
      [13, 32],
      [14, 39],
      [15, 42],
      [16, 62]
    ];

/*
    
    
    var plot = $.plot($("#placeholder3xx4"), [{
      label: "Raw_Lasers",
      data: d2,
      lines: {
        fillColor: "rgba(150, 202, 89, 0.12)"
      }, //#96CA59 rgba(150, 202, 89, 0.42)
      points: {
        fillColor: "#fff"
      }
    }], options);
    var plot = $.plot($("#placeholder3xx5"), [{
      label: "YAW",
      data: d3,
      lines: {
        fillColor: "rgba(150, 202, 89, 0.12)"
      }, //#96CA59 rgba(150, 202, 89, 0.42)
      points: {
        fillColor: "#fff"
      }
    }], options);

$(".Monmien").ionRangeSlider({
        type: "single",
        min: 0,
        max: 200,
        max_interval: 50
      });*/
