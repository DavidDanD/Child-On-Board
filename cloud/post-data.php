<?php
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

$servername = "localhost";

// REPLACE with your Database name
$dbname = "cdrmcrmy_esp32_sim800l";
// REPLACE with Database user
$username = "cdrmcrmy_dado";
// REPLACE with Database user password
$password = "Dado9092";

// Keep this API Key value to be compatible with the ESP32 code provided in the project page. If you change this value, the ESP32 sketch needs to match
$api_key_value = "tPmAT5Ab3j7F9";

$api_key = $value1 = $value2 = $value3 = $keys = "";
$values = "'";

if ($_SERVER["REQUEST_METHOD"] == "POST") {
    $api_key = test_input($_POST["api_key"]);
    if($api_key == $api_key_value) {
        $iter = 0;
        foreach($_POST as $key => $value) {
            if(test_input($key) == "api_key"){
                $iter += 1;
                continue;
            }
            if ( $iter == (count($_POST)-1) ){
                $keys .= test_input($key);
                $values .=  test_input($value) . "'";
            }
            else{
                $keys .= test_input($key) . ", ";
                $values .=  test_input($value) . "', '";
            } 
            $iter += 1;
        }
        // $keys = "value1, value2, value3";
        // $values = $tmp . ", '3', '4'";
        
        // Create connection
        $conn = new mysqli($servername, $username, $password, $dbname);
        // Check connection
        if ($conn->connect_error) {
            die("Connection failed: " . $conn->connect_error);
        } 
        
        $sql = "INSERT INTO Sensor (" . $keys . ")
        VALUES (" . $values . ")";
        
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