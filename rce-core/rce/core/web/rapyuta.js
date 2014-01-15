function run() {
    var canvas = document.getElementById("rapyuta");

    if (canvas.getContext){
        var context = canvas.getContext("2d");

        websocket = new WebSocket("ws://" + window.location.hostname + ":14014");
        websocket.onopen = function(event) { onOpen(canvas, context, event) };
        websocket.onclose = function(event) { onClose(canvas, context, event) };
        websocket.onmessage = function(event) { onMessage(canvas, context, event) };
        websocket.onerror = function(event) { onError(canvas, context, event) };
    }
}

function onOpen(canvas, context, event) {
    console.log("WebSocket opened");
}

function onClose(canvas, context, event) {
    console.log("WebSocket closed");
}

function onMessage(canvas, context, event) {
    console.log("WebSocket message received");
    draw(canvas, context, JSON.parse(event.data));
}

function onError(canvas, context, event) {
    console.log("WebSocket ERROR!");
}

function draw(canvas, context, endpoints) {
    context.clearRect(0, 0, canvas.width, canvas.height);

    context.fillStyle = "#88bce7";
    context.fillRect(100, (canvas.height / 2) - 20, canvas.width - 200, 40);

    context.font = "16px sans-serif";
    context.fillStyle = "#0075bf";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText("RoboEarth Cloud Engine", canvas.width / 2, canvas.height / 2);

    drawEPs(canvas, context, endpoints.container, canvas.height / 4);
    drawEPs(canvas, context, endpoints.robot, 3 * canvas.height / 4);
}

function drawEPs(canvas, context, endpoints, y) {
    var EP_W = 150;
    var EP_H = 100;

    var endpointsLength = Object.keys(endpoints).length;

    console.log(endpointsLength);

    var x = (canvas.width - endpointsLength * EP_W) / (endpointsLength + 1);
    var inc = x + EP_W;

    for(var cTag in endpoints){
        if (endpoints.hasOwnProperty(cTag)) {
            var nodes = endpoints[cTag];
            var nodesLength = nodes.length;

            context.save();

            context.translate(x, y - EP_H / 2);

            context.fillStyle = "#0075bf";
            context.fillRect(0, 0, EP_W, EP_H);

            context.font = "12px sans-serif";
            context.strokeStyle = context.fillStyle = "#ffffff";
            context.textAlign = "left";
            context.textBaseline = "middle";

            context.beginPath();

            context.fillText(cTag, 5, 10);

            for (var i = 0; i < nodesLength; ++i) {

                if (i == 0) {
                    context.moveTo(10, 17);
                } else {
                    context.moveTo(10, 10 + i * 15);
                }

                context.lineTo(10, 25 + i * 15);
                context.lineTo(17, 25 + i * 15);

                context.fillText(nodes[i], 20, 25 + i * 15);
            }

            context.stroke();

            context.restore();

            x += inc;
        }
    }
}
