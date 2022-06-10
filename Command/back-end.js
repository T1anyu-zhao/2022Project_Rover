var express = require('express');
var server = express();
server.get('/', function(req, res) {
 res.sendFile('/Users/Owner/Documents/GitHub/2022Project_Rover/Command/index.html');
});
server.get('/styles.css', function(req, res) {
 res.sendFile('/Users/Owner/Documents/GitHub/2022Project_Rover/Command/styles.css');
});
console.log('Server is running on port 8000');
server.listen(8000,'localhost');