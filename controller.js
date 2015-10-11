var controls = require("/root/beagle-copter/build/Release/controls.node");
var b = require('bonescript');
var io = require('socket.io');
var server = io.listen(5050); // Port number

var BLDC_PWM1 = 'P9_14';
var BLDC_PWM2 = 'P8_13';
var BLDC_PWM3 = 'P9_22';
var BLDC_PWM4 = 'P9_42';

var MAX_POSITION = 40;
var UPDATE_DUTY = 100; // 100ms

// period (T) = 1 / freq (F)
// 400Hz -> 1 / 400 = 0.0025 => period (T) = 2.5ms
// Pulse Width = period * duty_cycle -> 2.5 * 40% = 1ms (Low throttle), 2.5 * 80% = 2ms (Full throttle)
var freq = 400; // DJI E300 supports 30Hz ~ 450Hz
var duty_min = 0.4;
var position1 = 0;
var position2 = 0;
var position3 = 0;
var position4 = 0;
var increment = 0.1;

var throttle = 0;

var fail_over;

var connected = false;
// b.pinMode(BLDC_PWM1, b.OUTPUT);
// b.pinMode(BLDC_PWM2, b.OUTPUT);
// b.pinMode(BLDC_PWM3, b.OUTPUT);
// b.pinMode(BLDC_PWM4, b.OUTPUT);

setup();
controls.start();

server.sockets.on('connection', function(socket) {
    console.log("connected");
    connected = true;
	    
	socket.on('new packet', function(data) {
		updateDuty(data);
		console.log("received packet : ", data);
	});
	socket.on('disconnect', function() {
		console.log("disconnected");
		connected = false;
		//failOver();
	});

	if (connected) {
		//sendBatteryVoltage(socket);
	}

});

function sendBatteryVoltage(socket) {
    var value = b.analogRead('P9_40');
    var voltage = 12.4 * value * (1.8 / 1.13);

    socket.emit("new_message", "{ battery_voltage: " + voltage + "  }");
    setTimeout(sendBatteryVoltage, 1000, socket);
    console.log("voltage : " + voltage + "V");
}

function setup() {
/*
    b.analogWrite(BLDC_PWM1, duty_min, freq);
    b.analogWrite(BLDC_PWM2, duty_min, freq);
    b.analogWrite(BLDC_PWM3, duty_min, freq);
    b.analogWrite(BLDC_PWM4, duty_min, freq);
*/
    console.log("Duty Cycle: " + parseFloat(duty_min*100).toFixed(1) + " %");
}

function updateDuty(packet) {
    var value = JSON.parse(packet.toString());
    throttle = value.throttle;
    var duty_cycle = map(throttle, 0, 1023, 0.4, 0.8);
    // console.log("duty_cycle : " + duty_cycle * 100 + "%");
    //b.analogWrite(BLDC_PWM1, duty_cycle, freq);
    //b.analogWrite(BLDC_PWM2, duty_cycle, freq);
    //b.analogWrite(BLDC_PWM3, duty_cycle, freq);
    //b.analogWrite(BLDC_PWM4, duty_cycle, freq);
    controls.update(throttle, value.rudder, value.elevator, value.aileron);
}

function decreaseDuty() {
    if (!connected) {
        var duty_cycle = map(throttle, 0, 1023, 0.4, 0.8);
        // console.log("duty_cycle : " + duty_cycle * 100 + "%");
        b.analogWrite(BLDC_PWM1, duty_cycle, freq);
        b.analogWrite(BLDC_PWM2, duty_cycle, freq);
        b.analogWrite(BLDC_PWM3, duty_cycle, freq);
        b.analogWrite(BLDC_PWM4, duty_cycle, freq);
        throttle = throttle - 1;
    } else {
        clearInterval(fail_over);
    }
}

function failOver() {
	fail_over = setInterval(decreaseDuty, 100);
}

function map(val, in_min, in_max, out_min, out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
