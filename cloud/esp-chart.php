<!--
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

-->
<?php

$servername = "localhost";

// REPLACE with your Database name
$dbname = "cdrmcrmy_esp32_sim800l";
// REPLACE with Database user
$username = "cdrmcrmy_dado";
// REPLACE with Database user password
$password = "Dado9092";

// Create connection
$conn = new mysqli($servername, $username, $password, $dbname);
// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
} 

$sql = "SELECT id, temperature, weight, gyro, accelerometer, state,
    reading_time FROM Sensor order by reading_time desc limit 1";

$result = $conn->query($sql);
$row = $result->fetch_assoc();
switch ($row["state"]) {
    case "0":
        $row["state"] = "RESET";
        break;
    case "1":
        $row["state"] = "START";
        break;
    case "2":
        $row["state"] = "SOFT_ALARM";
        break;
    case "3":
        $row["state"] = "ALARM";
        break;
    case "4":
        $row["state"] = "SMS";
        break;
    case "5":
        $row["state"] = "ENGINE_OFF";
        break;
}



$sql = "SELECT id, temperature, weight, gyro,  accelerometer, state, reading_time FROM Sensor order by reading_time desc limit 40";

$result = $conn->query($sql);

while ($data = $result->fetch_assoc()){
    $sensor_data[] = $data;
}

$readings_time = array_column($sensor_data, 'reading_time');

for ($x=0; $x < sizeof($readings_time); $x++){
    $readings_time[$x] = explode(" ", $readings_time[$x])[1];
    // $readings_time[$x] = date("H:i:s", $readings_time[$x]);
}
// print_r($readings_time);

// $readings_time = array_column($sensor_data, 'reading_time');

// ******* Uncomment to convert readings time array to your timezone ********
$i = 0;
foreach ($readings_time as $reading){
    // Uncomment to set timezone to - 1 hour (you can change 1 to any number)
    $readings_time[$i] = date("Y-m-d H:i:s", strtotime("$reading + 9 hours"));
    // Uncomment to set timezone to + 4 hours (you can change 4 to any number)
    //$readings_time[$i] = date("Y-m-d H:i:s", strtotime("$reading + 4 hours"));
    $i += 1;
}


$temperature = json_encode(array_reverse(array_column($sensor_data, 'temperature')), JSON_NUMERIC_CHECK);
$weight = json_encode(array_reverse(array_column($sensor_data, 'weight')), JSON_NUMERIC_CHECK);
$gyro = json_encode(array_reverse(array_column($sensor_data, 'gyro')), JSON_NUMERIC_CHECK);
$accelerometer = json_encode(array_reverse(array_column($sensor_data, 'accelerometer')), JSON_NUMERIC_CHECK);
$state = json_encode(array_reverse(array_column($sensor_data, 'state')), JSON_NUMERIC_CHECK);
$reading_time = json_encode(array_reverse($readings_time), JSON_NUMERIC_CHECK);

$result->free();
$conn->close();
?>

<!DOCTYPE html>
<html lang="en">
<head>
    <title>Child On Board</title>
    <link rel="shortcut icon" href="/ChildOnBoardLogo.ico">
    <style>
    * {
      box-sizing: border-box;
    }
    
    .column {
      float: left;
      width: 33.33%;
      padding: 5px;
    }
    
    /* Clearfix (clear floats) */
    .row::after {
      content: "";
      clear: both;
      display: table;
    }
    </style>
</head>
<body>
    <div class="row">
      <div class="column">
        <img src="ChildOnBoard.png" alt="logo" style="width:100%">
      </div>
      <div class="column">
        <h1 style="text-align:center;">Status</h1>
        <p style="text-align:center;">Temperatue: <?php echo $row["temperature"]; ?></p>
        <p style="text-align:center;">Weight: <?php echo $row["weight"]; ?></p>
        <p style="text-align:center;">gyro: <?php echo $row["gyro"]; ?></p>
        <p style="text-align:center;">accelerometer: <?php echo $row["accelerometer"]; ?></p>
        <p style="text-align:center;">state: <?php echo $row["state"]; ?></p>
        <p style="text-align:center;">Sample Date: <?php echo explode(" ", $row["reading_time"])[1]; ?></p>
      </div>
      <div class="column">
        <img src="ChildOnBoard.png" alt="logo" style="width:100%">
      </div>
    </div>
</body>
<meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://code.highcharts.com/highcharts.js"></script>
  <style>
    body {
      min-width: 310px;
    	max-width: 1280px;
    	height: 500px;
      margin: 0 auto;
    }
    h2 {
      font-family: Arial;
      font-size: 2.5rem;
      text-align: center;
    }
  </style>
  <body>
    <h2>ESP Weather Station</h2>
    <div id="chart-temperature" class="container" style="height: 400px; width: 48%; display:inline-block;"></div>
    <div id="chart-weight" class="container" style="height: 400px; width: 48%; display:inline-block;"></div>
    <div id=""></div>
    <div id="chart-gyro" class="container" style="height: 400px; width: 48%; display:inline-block;"></div>
    <div id="chart-accelerometer" class="container" style="height: 400px; width: 48%; display:inline-block;"></div>
<script>

var temperature = <?php echo $temperature; ?>;
var weight = <?php echo $weight; ?>;
var gyro = <?php echo $gyro; ?>;
var accelerometer = <?php echo $accelerometer; ?>;
var state = <?php echo $state; ?>;
var reading_time = <?php echo $reading_time; ?>;

var chartT = new Highcharts.Chart({
  chart:{ renderTo : 'chart-temperature'},
  title: { text: 'Temperature' },
  series: [{
    showInLegend: false,
    data: temperature
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    },
    series: { color: '#059e8a' }
  },
  xAxis: { 
    type: 'datetime',
    categories: reading_time
  },
  yAxis: {
    title: { text: 'Temperature (Celsius)' }
    //title: { text: 'Temperature (Fahrenheit)' }
  },
  credits: { enabled: false }
});

var chartW = new Highcharts.Chart({
  chart:{ renderTo:'chart-weight' },
  title: { text: 'Weight' },
  series: [{
    showInLegend: false,
    data: weight
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    }
  },
  xAxis: {
    type: 'datetime',
    dateTimeLabelFormats: { hour: '%H:%M' },
    categories: reading_time
  },
  yAxis: {
    title: { text: 'Weight' }
  },
  credits: { enabled: false }
});


var chartG = new Highcharts.Chart({
  chart:{ renderTo:'chart-gyro' },
  title: { text: 'Gyroscope' },
  series: [{
    showInLegend: false,
    data: gyro
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    },
    series: { color: '#18009c' }
  },
  xAxis: {
    type: 'datetime',
    categories: reading_time
  },
  yAxis: {
    title: { text: 'Gyroscope' }
  },
  credits: { enabled: false }
});


var chartA = new Highcharts.Chart({
  chart:{ renderTo:'chart-accelerometer' },
  title: { text: 'Accelerometer' },
  series: [{
    showInLegend: false,
    data: accelerometer
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    },
    series: { color: '#18009c' }
  },
  xAxis: {
    type: 'datetime',
    categories: reading_time
  },
  yAxis: {
    title: { text: 'Accelerometer' }
  },
  credits: { enabled: false }
});

</script>
</body>
</html>