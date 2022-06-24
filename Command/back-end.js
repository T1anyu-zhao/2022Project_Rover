var express = require('express');
var server = express();
var bodyParser = require('body-parser');
server.use(bodyParser.json());
server.use(bodyParser.urlencoded({ extended: true }));
var mysql = require('mysql');
var http = require('http');

const con = mysql.createConnection({
    host: 'localhost',
    user: 'root',
    password: 'toor',
    database: 'rover'
});

con.connect(function(err) {
    if (err){ 
        console.log("Could not connect to the database.");
        throw err;
    }
    console.log("Connected to Database!");
}); 

/*
var control = "CREATE TABLE control (mode VARCHAR(255))";
con.query(control, function (err, result) {
    if (err) throw err;
    console.log("control table created!");
});
*/

// Client's browser performs GET request to ask server to display HTML web page
server.get('/', function(req, res) {
    res.sendFile('/Users/Owner/Documents/GitHub/2022Project_Rover/Command/index.html');
});

// index.html requires styles.css so you have to add a GET request in node.js
server.get('/styles.css', function(req, res) {
    res.sendFile('/Users/Owner/Documents/GitHub/2022Project_Rover/Command/styles.css');
});

// when a user clicks a button on the web app, data is sent to the server using a POST request
server.post("/datastream", function (req, res) {
    var click = req.body.data;
    res.send(click);
    console.log(click);
    var control = "UPDATE control SET mode = '"+click+"'";
    con.query(control, function (err, result) {
        if (err) throw err;
        console.log("driving mode updated"); 
    });
});
/*
con.end((err) => {
    // The connection is terminated gracefully
    // Ensures all remaining queries are executed
    // Then sends a quit packet to the MySQL server.
}); 
*/

// testing 
//console.log(movement);

/* esp32 accesses the data using a GET request
server.get("/datastream", function (req, res) {
    res.send(req.body.data);
    console.log('esp32 get request');
    res.end('Hello');
});
*/

console.log('Server is running on port 8000');
server.listen(8000,'0.0.0.0');