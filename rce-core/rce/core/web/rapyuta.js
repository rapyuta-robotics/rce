var FONT_SIZE = 16; // has to be divisible by 2!

// Middle bar
var BAR_H = 80;

// Endpoint
var EP_W = 250;
var EP_H = 100;
var BORDER = 5;
var X_OFFSET = 15; // has to be divisible by 3!
var X_MARGIN = 3; // space between line end & text begin

// Traffic arrows
var ARROW_W = 10;
var ARROWHEAD_W = 30;
var ARROW_H = 70;
var ARROWHEAD_H = 25;

var X_TEXT = BORDER + X_OFFSET;
var X_START = BORDER + X_OFFSET / 3 + 0.5;
var X_END = X_TEXT - X_MARGIN;

var Y_START = BORDER + FONT_SIZE / 2;
var Y_CONTAINER_NODE_INCREMENT = FONT_SIZE / 2 + 0.5;
var Y_NODE_NODE_INCREMENT = FONT_SIZE;

var X_ARROW = ARROW_W / 2;
var X_ARROWHEAD = ARROWHEAD_W / 2;
var Y_ARROW = ARROW_H / 2;
var Y_ARROWHEAD = Y_ARROW - ARROWHEAD_H;

function run() {
    var canvas = document.getElementById("rapyuta");

    if (canvas.getContext){
        var context = canvas.getContext("2d");
        var websocket = new WebSocket("ws://" + window.location.hostname + ":14014");

        websocket.onclose = function(event) { onClose(canvas, context, event) };
        websocket.onmessage = function(event) { onMessage(canvas, context, event) };
        websocket.onerror = function(event) { onError(canvas, context, event) };
    }
}

function onClose(canvas, context, event) {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.font = "20px sans-serif";
    context.fillStyle = "#ff0000";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText("disconnected", canvas.width / 2, canvas.height / 2);
}

function onMessage(canvas, context, event) {
    draw(canvas, context, JSON.parse(event.data));
}

function onError(canvas, context, event) {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.font = "20px bold sans-serif";
    context.fillStyle = "#ff0000";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText("ERROR", canvas.width / 2, canvas.height / 2);
}

function draw(canvas, context, endpoints) {
    context.clearRect(0, 0, canvas.width, canvas.height);

    context.strokeStyle = "#88bce7";
    context.strokeRect(100, (canvas.height - BAR_H) / 2, canvas.width - 200, BAR_H);

    context.font = "20px sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillStyle = "#0075bf";
    context.fillText("RoboEarth Cloud Engine", canvas.width / 2, canvas.height / 2);

    drawContainers(canvas, context, endpoints.container);
    drawRobots(canvas, context, endpoints.robot);
}

function drawContainers(canvas, context, containers) {
    var numContainers = containers.length;

    var x = Math.round((canvas.width - numContainers * EP_W) / (numContainers + 1));
    var y = Math.round(3 * canvas.height / 16 - EP_H / 2);
    var y_traffic = Math.round(((canvas.height - BAR_H) / 2 - (y + EP_H)) / 2 + EP_H);

    var increment = x + EP_W;

    for (var i = 0; i < numContainers; ++i, x += increment) {
        var container = containers[i]
        var nodes = container[2];
        var numNodes = nodes.length;

        var yi = Y_START;

        context.save();
        context.translate(x, y);

        context.font = FONT_SIZE + "px sans-serif";
        context.textAlign = "center";
        context.textBaseline = "middle";

        context.fillStyle = "#0075bf";
        context.fillRect(0, 0, EP_W, EP_H);

        context.fillStyle = "#202020";
        context.fillText("Virtual Machine " + i, EP_W / 2, -Y_START);

        context.textAlign = "left";
        context.strokeStyle = context.fillStyle = "#ffffff";
        context.beginPath();
        context.fillText(container[0], BORDER, yi);

        yi += Y_CONTAINER_NODE_INCREMENT;

        for (var j = 0; j < numNodes; ++j) {
            context.moveTo(X_START, yi + 0.5); // '+' == one pixel margin

            if (j == 0) {
                yi += Y_NODE_NODE_INCREMENT / 2;
            } else {
                yi += Y_NODE_NODE_INCREMENT;
            }

            context.lineTo(X_START, yi);
            context.lineTo(X_END, yi);
            context.fillText(nodes[j], X_TEXT, yi);
        }

        context.stroke();

        context.save();
        context.translate(EP_W / 2, y_traffic);

        context.font = FONT_SIZE + "px sans-serif";
        context.textBaseline = "middle";

        context.strokeStyle = context.fillStyle = "#00cc00";
        drawArrow(context, 1);
        context.textAlign = "left";
        context.fillText(formatBandwidth(container[1][0]), ARROWHEAD_W + 10, 0);

        context.strokeStyle = context.fillStyle = "#cc0000";
        drawArrow(context, -1);
        context.textAlign = "right";
        context.fillText(formatBandwidth(container[1][1]), -(ARROWHEAD_W + 10), 0);

        context.restore();

        context.restore();
    }
}

function drawRobots(canvas, context, robots) {
    var numRobots = robots.length;

    var x = Math.round((canvas.width - numRobots * EP_W) / (numRobots + 1));
    var y = Math.round(13 * canvas.height / 16 - EP_H / 2);
    var y_traffic = Math.round((y - (canvas.height + BAR_H) / 2) / 2);

    var inc = x + EP_W;

    for (var i = 0; i < numRobots; ++i) {
        var robot = robots[i];

        context.save();
        context.translate(x + i * inc, y);

        context.font = FONT_SIZE + "px sans-serif";
        context.textAlign = "center";
        context.textBaseline = "middle";

        context.fillStyle = "#88bce7";
        context.fillRect(0, 0, EP_W, EP_H);

        context.fillStyle = "#202020";
        context.fillText("Robot Client " + i, EP_W / 2, EP_H + Y_START);

        context.fillStyle = "#000000";
        context.fillText(robot[0], EP_W / 2, EP_H / 2);

        context.save();
        context.translate(EP_W / 2, -y_traffic);

        context.font = FONT_SIZE + "px sans-serif";
        context.textBaseline = "middle";

        context.strokeStyle = context.fillStyle = "#00cc00";
        drawArrow(context, -1);
        context.textAlign = "right";
        context.fillText(formatBandwidth(robot[1][0]), -(ARROWHEAD_W + 10), 0);

        context.strokeStyle = context.fillStyle = "#cc0000";
        drawArrow(context, 1);
        context.textAlign = "left";
        context.fillText(formatBandwidth(robot[1][1]), ARROWHEAD_W + 10, 0);

        context.restore();

        context.restore();
    }
}

function drawArrow(context, sign) {
        context.save();
        context.translate(sign * X_ARROWHEAD, 0);
        context.lineWidth = 2;

        context.beginPath();
        context.moveTo(-X_ARROW, -sign * Y_ARROW);
        context.lineTo(-X_ARROW, sign * Y_ARROWHEAD);
        context.lineTo(-X_ARROWHEAD, sign * Y_ARROWHEAD);
        context.lineTo(0, sign * Y_ARROW);
        context.lineTo(X_ARROWHEAD, sign * Y_ARROWHEAD);
        context.lineTo(X_ARROW, sign * Y_ARROWHEAD);
        context.lineTo(X_ARROW, -sign * Y_ARROW);
        context.lineTo(-X_ARROW, -sign * Y_ARROW);

        context.stroke();

        context.restore();
}

function formatBandwidth(bw) {
    var ending;

    if (bw / 1000 > 1) {
        bw = bw / 1000;
        ending = " kB/s";
    } else {
        ending = " B/s";
    }

    if (bw / 10 < 1) {
        return bw.toFixed(2) + ending;
    } else {
        return bw.toFixed(1) + ending;
    }
}
