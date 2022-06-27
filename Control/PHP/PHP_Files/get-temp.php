<?php
   $servername = "localhost";
   $username = "ESP32";
   $password = "esp32io.com";
   $database_name = "db_esp32";

   //include_once('esp-database.php');

   $action = $id = $name = $gpio = $state = "";//change

   if ($_SERVER["REQUEST_METHOD"] == "GET") {
       //$action = test_input($_GET["action"]);
       //if ($action == "outputs_state") {
           //$board = test_input($_GET["board"]);
            $result = getAllOutputs();
            while ($row = $result->fetch_assoc()) {
               $values[$row["temp_id"]] = $row["temp_value"];
            }
            $result = json_encode($values);
            echo $result;   
       //}
       
      //  else {
      //      echo "Invalid HTTP request.";
      //  }
   }

   function test_input($data) {
       $data = trim($data);
       $data = stripslashes($data);
       $data = htmlspecialchars($data);
       return $data;
   }
   
   function getAllOutputs() {
      global $servername, $username, $password, $database_name;

      // Create connection
      $conn = new mysqli($servername, $username, $password, $database_name);
      // Check connection
      if ($conn->connect_error) {
          die("Connection failed: " . $conn->connect_error);
      }

      $sql = "SELECT * FROM tbl_temp";
      if ($result = $conn->query($sql)) {
      //    while($row = mysql_fetch_array($result)) {
      //       echo $row['column_name']; // Print a single column data
      //       echo print_r($row);       // Print the entire row data
      //   }
         return $result;
         echo $result;
      }
      else {
          return false;
      }
      $conn->close();
  }
?>