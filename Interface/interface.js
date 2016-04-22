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


var msg = new ROSLIB.Message({
    data : "yesouiok"
});

  console.log("Publishing cmd_vel");
  cmdVel.publish(twist);

  console.log("Publishing data");
  exampleTopic.publish(msg);   

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

 data1=data.local.position.x;
 data2=data.local.position.y;

plotLocalZ(data.header.seq/25, data.local.position.z)
plotLocalXY(data1,data2);


 });



   
function changePose(id, position){
	var elem = $(id);
	elem.find('.datax').text(position.x.toFixed(3));
	elem.find('.datay').text(position.y.toFixed(3));
	elem.find('.dataz').text(position.z.toFixed(3));
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
  elem.find('.datax1').text(raw.status[0].toFixed(3));
  elem.find('.datax2').text(raw.status[1].toFixed(3));
  elem.find('.datay1').text(raw.status[2].toFixed(3));
  elem.find('.datay2').text(raw.status[3].toFixed(3));
  elem.find('.dataz1').text(raw.status[4].toFixed(3));
  elem.find('.dataz2').text(raw.status[5].toFixed(3));

}

$("button.motorstop").click(test)
function test() {
    alert(4);
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


