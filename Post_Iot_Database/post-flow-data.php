<?php
$servername = "localhost";

// REPLACE with your Database name
$dbname = "id21802268_rewanaiot";
// REPLACE with Database user
$username = "id21802268_rewanaiot";
// REPLACE with Database user password
$password = "Rewanaiot123,";

// Keep this API Key value to be compatible with the ESP32 code provided in the project page. 
// If you change this value, the ESP32 sketch needs to match
$api_key_value = "BPmAT5Ab3j7F9";

$api_key = $sensor = $location = $flowrate = $debit = "";

if ($_SERVER["REQUEST_METHOD"] == "POST") {
    $api_key = test_input($_POST["api_key"]);
    if($api_key == $api_key_value) {
        $sensor = test_input($_POST["sensor"]);
        $location = test_input($_POST["location"]);
        $flowrate = test_input($_POST["flowrate"]);
        $debit = test_input($_POST["debit"]);
        
        // Create connection
        $conn = new mysqli($servername, $username, $password, $dbname);
        // Check connection
        if ($conn->connect_error) {
            die("Connection failed: " . $conn->connect_error);
        } 
        
        $sql = "INSERT INTO SensorFlowWater (sensor, location, flowrate, debit)
        VALUES ('" . $sensor . "', '" . $location . "', '" . $flowrate . "', '" . $debit . "')";
        
        if ($conn->query($sql) === TRUE) {
            echo "New record created successfully";
        } 
        else {
            echo "Error: " . $sql . "<br>" . $conn->error;
        }
    
        $conn->close();
    }
    else {
        echo "Wrong API Key provided.";
    }

}
else {
    echo "No data posted with HTTP POST.";
}

function test_input($data) {
    $data = trim($data);
    $data = stripslashes($data);
    $data = htmlspecialchars($data);
    return $data;
}
