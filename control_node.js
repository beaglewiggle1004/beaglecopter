var controls = require("./build/Release/controls.node");
var thr = 0, rud = 0, ele = 0, ail = 0;

controls.start();

setInterval(update, 1);

function update() {
	controls.update(thr++, rud++, ele++, ail++);
}
