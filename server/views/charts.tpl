<!DOCTYPE HTML><html>
<!-- Rui Santos - Complete project details at https://RandomNerdTutorials.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files.
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software. -->
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://code.highcharts.com/highcharts.js"></script>
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
      <div id="heart-beat" class="heart-beat-info">
        Heart Beat Rate: 149 bpm
      </div>
      <div id="oxygentation" class="oxygentation-info">
        Sp02: 98%
      </div>
    </div>
    <div class="line2">
      <div class="clock-info">00:00:00</div>
    </div>
    <div class=line3>
      <div class="date-info">Monday, Novemeber 9 2021</div>
    </div>
    <div class=line4>
      <div class="activity-info">Activity: Sitting</div>
      <div class="battery-info">Battery: 3.8V</div>
    </div>
    <div class="line5">
      <div class="steps-info">Step: 18,390</div>
    </div>
  </div>
  <div id="watch-report" class="watch-button-record">
    <button type="button" class="record-button">Record Data</button>
  </div>
  <div id="chart-temperature" class="container"></div>
  <div id="chart-humidity" class="container"></div>
  <div id="chart-pressure" class="container"></div>
</body>
<script>
var chartT = new Highcharts.Chart({
  chart:{ renderTo : 'chart-temperature' },
  title: { text: 'BME280 Temperature' },
  series: [{
    showInLegend: false,
    data: []
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    },
    series: { color: '#059e8a' }
  },
  xAxis: { type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' }
  },
  yAxis: {
    title: { text: 'Temperature (Celsius)' }
    //title: { text: 'Temperature (Fahrenheit)' }
  },
  credits: { enabled: false }
});
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var x = (new Date()).getTime(),
          y = parseFloat(this.responseText);
      //console.log(this.responseText);
      if(chartT.series[0].data.length > 40) {
        chartT.series[0].addPoint([x, y], true, true, true);
      } else {
        chartT.series[0].addPoint([x, y], true, false, true);
      }
    }
  };
  xhttp.open("GET", "/temperature", true);
  xhttp.send();
}, 30000 ) ;

var chartH = new Highcharts.Chart({
  chart:{ renderTo:'chart-humidity' },
  title: { text: 'BME280 Humidity' },
  series: [{
    showInLegend: false,
    data: []
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    }
  },
  xAxis: {
    type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' }
  },
  yAxis: {
    title: { text: 'Humidity (%)' }
  },
  credits: { enabled: false }
});
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var x = (new Date()).getTime(),
          y = parseFloat(this.responseText);
      //console.log(this.responseText);
      if(chartH.series[0].data.length > 40) {
        chartH.series[0].addPoint([x, y], true, true, true);
      } else {
        chartH.series[0].addPoint([x, y], true, false, true);
      }
    }
  };
  xhttp.open("GET", "/humidity", true);
  xhttp.send();
}, 30000 ) ;

var chartP = new Highcharts.Chart({
  chart:{ renderTo:'chart-pressure' },
  title: { text: 'BME280 Pressure' },
  series: [{
    showInLegend: false,
    data: []
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    },
    series: { color: '#18009c' }
  },
  xAxis: {
    type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' }
  },
  yAxis: {
    title: { text: 'Pressure (hPa)' }
  },
  credits: { enabled: false }
});
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var x = (new Date()).getTime(),
          y = parseFloat(this.responseText);
      //console.log(this.responseText);
      if(chartP.series[0].data.length > 40) {
        chartP.series[0].addPoint([x, y], true, true, true);
      } else {
        chartP.series[0].addPoint([x, y], true, false, true);
      }
    }
  };
  xhttp.open("GET", "/pressure", true);
  xhttp.send();
}, 30000 ) ;
</script>
</html>