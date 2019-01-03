/* This is an instantiation of a server that we designed:
-- to run on a local machine (laptop)
-- creates a web server that will communicate with the local USB port
-- serves a webpage to a client that will plot using a javscript chart library
-- server side
October 2018 -- Emily Lam
*/


// Modules
var SerialPort = require('serialport');
var Readline = require('@serialport/parser-readline')
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

// Open serial port
var port = new SerialPort('/dev/tty.SLAB_USBtoUART', {baudRate: 115200});

// Read data from serial
var read_data;
var msg_us1;
var msg_ir;
var msg_lidar;
var msg_wheel;

var myArray;
const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', function (data) {
  //read_data = data;
  MyArray = data.split(" ");
  console.log('Read:', MyArray[0], MyArray[1]);

  if(MyArray[0] == "Ultrasonic")
    msg_us1 = parseFloat(MyArray[1]);
  if(MyArray[0] == "IR")
    msg_ir = parseFloat(MyArray[1]);
  if(MyArray[0] == "Lidar")
    msg_lidar = parseFloat(MyArray[1]);
  if(MyArray[0] == "Wheel")
    msg_wheel = parseFloat(MyArray[1]);
    // Convert to float
  io.emit('Ultrasonic1', msg_us1);
  io.emit('Infrared', msg_ir);
  io.emit('Lidar', msg_lidar);
  io.emit('Wheel', msg_wheel);
});

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});