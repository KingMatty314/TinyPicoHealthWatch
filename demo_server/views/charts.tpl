<!DOCTYPE HTML><html>
<!-- Rui Santos - Complete project details at https://RandomNerdTutorials.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files.
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software. -->
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://code.highcharts.com/highcharts.js"></script>
  <script  src="https://cdnjs.cloudflare.com/ajax/libs/paho-mqtt/1.0.1/mqttws31.min.js" type="text/javascript"></script>
  <style>
    body {
      min-width: 310px;
    	max-width: 800px;
    	height: 400px;
      margin: 0 auto;
    }
    h2 {
      font-family: Arial;
      font-size: 2.5rem;
      text-align: center;
    }
    .watch-info{
      min-width: 310px;
      max-width: 800px;
      min-height: 100;
      max-height: 200;
      margin: 30px;
      background-color: darkgrey;
      text-align: center;
    }
    .watch-button-record{
      margin: auto;
      text-align: center;
    }
    .heart-beat-info{
      margin: 30px 30px 30px 30px;
      color:white;
      display: inline-block;
      font: 1.5em sans-serif;
    }
    .oxygentation-info{
      margin: 30px 30px 30px 30px;
      color:white;
      display: inline-block;
      font: 1.5em sans-serif;
    }
    .clock-info{
      margin: 30px 0px 30px 30px;
      color:white;
      display: inline-block;
      font: 4em sans-serif;
    }
    .date-info{
      margin: 30px 30px 30px 30px;
      color:white;
      display: inline-block;
      font: 1.5em sans-serif;
    }
    .activity-info{
      margin: 30px 0px 30px 30px;
      color:white;
      display: inline-block;
      font: 1.5em sans-serif;
    }
    .battery-info{
      margin: 30px 0px 30px 30px;
      color:white;
      display: inline-block;
      font: 1.5em sans-serif;
    }
    .steps-info{
      margin: 30px 0px 30px 30px;
      color:white;
      display: inline-block;
      font:1.5em sans-serif;
    }
    .record-button{
      background-color: #4b4d4b; /* Green */
      border: none;
      color: white;
      padding: 15px 32px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
      transition-duration: 0.4s;
      margin-bottom: 50px;
    }
    .record-button:hover{
      background-color: #8f918f; /* Green */
      border: none;
      color: white;
      padding: 15px 32px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
      transition-duration: 0.4s;
      margin-bottom: 50px;
    }
    .record-button:active{
      background-color: #b3b4b3; /* Green */
      border: none;
      color: white;
      padding: 15px 32px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
      transition-duration: 0.4s;
      margin-bottom: 50px;
    }
  </style>
</head>
<body>
  <h2>ESP Health Watch</h2>
  <div id="watch-face" class="watch-info">
    <div class="line1">
      <div id="heartbeat" class="heart-beat-info">
        Heart Beat Rate: 149 bpm
      </div>
      <div id="oxygentation" class="oxygentation-info">
        Sp02: 98%
      </div>
    </div>
    <div class="line2">
      <div class="clock-info" id="clock">00:00:00</div>
    </div>
    <div class=line3>
      <div class="date-info" id="date">Monday, Novemeber 9 2021</div>
    </div>
    <div class=line4>
      <div class="activity-info" id="activity">Activity: Sitting</div>
      <div class="battery-info" id="battery">Battery: 3.8V</div>
    </div>
    <div class="line5">
      <div class="steps-info" id="steps">Step: 18,390</div>
    </div>
  </div>
  <div id="watch-report" class="watch-button-record">
    <button type="button" class="record-button">Record Data</button>
  </div>
  <div id="chart-accel" class="container"></div>
  <div id="chart-red" class="container"></div>
  <div id="chart-ir" class="container"></div>
</body>

<script>
// Correct date
var date = new Date(); 
document.getElementById("date").innerHTML = "Thursday, December " + date.getDate() + " " + date.getFullYear();

/// Chart Stuff
var chartAccel = new Highcharts.Chart({
  chart:{ renderTo : 'chart-accel' },
  title: { text: 'Accelerometer' },
  series: [{
    showInLegend: false,
    data: []
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: false }
    },
    series: { color: '#059e8a' }
  },
  xAxis: { type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' }
  },
  yAxis: {
    title: { text: 'Accel Magnitude' }
  },
  credits: { enabled: false }
});

var chartRed = new Highcharts.Chart({
  chart:{ renderTo:'chart-red' },
  title: { text: 'Red Light' },
  series: [{
    showInLegend: false,
    data: []
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: false }
    }
  },
  xAxis: {
    type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' }
  },
  yAxis: {
    title: { text: 'Voltage (mV)' }
  },
  credits: { enabled: false }
});

var chartIR = new Highcharts.Chart({
  chart:{ renderTo:'chart-ir' },
  title: { text: 'IR Light' },
  series: [{
    showInLegend: false,
    data: []
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: false }
    },
    series: { color: '#18009c' }
  },
  xAxis: {
    type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' }
  },
  yAxis: {
    title: { text: 'Voltage (mV)' }
  },
  credits: { enabled: false }
});

// MQTT Client
var reconnectTimeout = 2000;
var host = '127.0.0.1';
var port = 80;
// Create a client instace
client  = new Paho.MQTT.Client(host, port, '', 'webpage');
// set callback handlers
client.onConnectLost = onConnectionLost;
client.onMessageArrived = onMessageArrived;
// Connect to broker
client.connect({onSuccess:onConnect});
// called when the client connects
function onConnect(){
  console.log('onConnect');
  client.subscribe('esp32watch/health/heartbeat');
  client.subscribe('esp32watch/health/sp02');
  client.subscribe('esp32watch/health/steps');
  client.subscribe('esp32watch/health/activity');
  client.subscribe('esp32watch/data/battery');
  client.subscribe('esp32watch/data/pedometer/accel');
  client.subscribe('esp32watch/data/oximeter/red');
  client.subscribe('esp32watch/data/oximeter/ir');
};

// called when the client losses its connection
function onConnectionLost(responseObject){
  if (responseObject.errorCode !== 0){
    console.log("onConnectionLost:" + responseObject.errorMessage);
  }
};

// called when a message arrives
function onMessageArrived(message){
  console.log("Destination:" + message.destinationName);
  console.log("Message:" + message.payloadString);
  if (message.destinationName == 'esp32watch/health/heartbeat'){

  } else if (message.destinationName == 'esp32watch/health/sp02'){
    document.getElementById("heartbeat").innerHTML = "Heart Beat Rate: " + message.payloadString + " bpm";
  } else if (message.destinationName == 'esp32watch/health/steps'){
    document.getElementById("steps").innerHTML = "Steps: " + message.payloadString;
  } else if (message.destinationName == 'esp32watch/health/sp02'){
    document.getElementById("oxygentation").innerHTML = "Sp02: " + message.payloadString + "%";
  } else if (message.destinationName == 'esp32watch/health/activity'){
    document.getElementById("activity").innerHTML = "Activity: " + message.payloadString;
  } else if (message.destinationName == 'esp32watch/data/battery'){
    document.getElementById("battery").innerHTML = "Battery: " + message.payloadString + " V";
  } else if (message.destinationName == 'esp32watch/data/pedometer/accel'){
    var x = (new Date()).getSeconds();
    var y = parseFloat(message.payloadString);
    var shift = series[0].data.length > 40;
    chartAccel.series[0].addPoint([x, y], true, true, shift);
    
  } else if (message.destinationName == 'esp32watch/data/oximeter/red'){
    var x = (new Date()).getSeconds();
    var y = parseFloat(message.payloadString);
    var shift = series[0].data.length > 40;
      chartRed.series[0].addPoint([x, y], true, false, shift);
  } else if (message.destinationName == 'esp32watch/data/oximeter/ir'){
    var x = (new Date()).getSeconds();
    var y = parseFloat(message.payloadString);
    var shift = series[0].data.length > 40
    chartIR.series[0].addPoint([x, y], true, false, shift);
  } else {
    console.error("Bad DestinationName");
  }
};

setInterval(function(){
  var time = new Date();
  document.getElementById("clock").innerHTML = time.getHours() + ":" + time.getMinutes() + ":" + time.getSeconds()
}, 500);

</script>
</html>