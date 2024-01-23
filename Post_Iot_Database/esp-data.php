<!DOCTYPE html>
<html>
<head>
 <style>
  
  table {
   border-collapse: collapse;
   width: 100%;
   margin-bottom: 1em;
   color: #333333;
   font-family: Arial, sans-serif;
   font-size: 14px;
   text-align: left;
   background-color: #F5F5F5;
  }

  table td, table th {
   padding: 8px;
   border: 1px solid #DDDDDD;
  }

  table th {
   background-color: #B3B3B3;
   color: #FFFFFF;
   font-weight: bold;
  }

  table tr:nth-child(even) {
   background-color: #EFEFEF;
  }

  table tr:nth-child(odd) {
   background-color: #FFFFFF;
  }

    h1 {
    text-align: center;
  }
 </style>
</head>  
<body>
<?php
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-mysql-database-php/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

$servername = "localhost";

// REPLACE with your Database name
$dbname = "Database name";
// REPLACE with Database user
$username = "Database user";
// REPLACE with Database user password
$password = "Database user password";

// Create connection
$conn = new mysqli($servername, $username, $password, $dbname);
// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
} 

$sql = "SELECT id, sensor, location, volt, turbiValue, reading_time FROM SensorTurbidity ORDER BY id DESC";

echo '<h1> Turbidity Database </h1>
      <table cellspacing="5" cellpadding="5">
      <tr> 
        <td>ID</td> 
        <td>Sensor</td> 
        <td>Location</td> 
        <td>Volt(V)</td> 
        <td>Turbidity Value(NTU)</td> 
        <td>Timestamp</td> 
      </tr>';
      
if ($result = $conn->query($sql)) {
    while ($row = $result->fetch_assoc()) {
        $row_id = $row["id"];
        $row_sensor = $row["sensor"];
        $row_location = $row["location"];
        $row_volt = $row["volt"];
        $row_turbiValue = $row["turbiValue"];
        $row_reading_time = $row["reading_time"];
        // Uncomment to set timezone to - 1 hour (you can change 1 to any number)
        //$row_reading_time = date("Y-m-d H:i:s", strtotime("$row_reading_time - 1 hours"));
      
        // Uncomment to set timezone to + 7 hours (you can change 7 to any number)
        $row_reading_time = date("Y-m-d H:i:s", strtotime("$row_reading_time + 7 hours"));
      
        echo '<tr> 
                <td>' . $row_id . '</td> 
                <td>' . $row_sensor . '</td> 
                <td>' . $row_location . '</td> 
                <td>' . $row_volt . '</td> 
                <td>' . $row_turbiValue . '</td>
                <td>' . $row_reading_time . '</td> 
              </tr>';
    }
    $result->free();
}

$conn->close();
?> 
</table>
</body>
</html>
