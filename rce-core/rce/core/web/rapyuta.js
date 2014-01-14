function run() {
    var canvas = document.getElementById("rapyuta");

    if (canvas.getContext){
        var ctx = canvas.getContext("2d");

        websocket = new WebSocket("ws://" + window.location.hostname + ":14014");
        websocket.onopen = function(evt) { onOpen(canvas, ctx, evt) };
        websocket.onclose = function(evt) { onClose(canvas, ctx, evt) };
        websocket.onmessage = function(evt) { onMessage(canvas, ctx, evt) };
        websocket.onerror = function(evt) { onError(canvas, ctx, evt) };
    }
}

function onOpen(canvas, ctx, evt) {
    console.log("WebSocket opened");
}

function onClose(canvas, ctx, evt) {
    console.log("WebSocket closed");
}

function onMessage(canvas, ctx, evt) {
    console.log("WebSocket message received");
    draw(canvas, ctx, JSON.parse(evt.data));
}

function onError(canvas, ctx, evt) {
    console.log("WebSocket ERROR!");
}

function draw(canvas, ctx, endpoints) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.fillStyle = "#88bce7";
    ctx.fillRect(100, (canvas.height / 2) - 20, canvas.width - 200, 40);

    ctx.font = "16px sans-serif";
    ctx.fillStyle = "#0075bf";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("RoboEarth Cloud Engine", canvas.width / 2, canvas.height / 2);

    drawEPs(canvas, ctx, endpoints.container, canvas.height / 4);
    drawEPs(canvas, ctx, endpoints.robot, 3 * canvas.height / 4);
}

function drawEPs(canvas, ctx, eps, y) {
    var EP_W = 150;
    var EP_H = 100;

    if (eps.length) {
        var inc = (canvas.width - eps.length * EP_W) / (eps.length + 1) + EP_W;
        var x = inc - EP_W / 2;

        eps.map(function(ep) {
            ctx.save();

            ctx.translate(x, y);

            ctx.fillStyle = "#88bce7";
            ctx.fillRect(-EP_W / 2, - EP_H / 2, EP_W, EP_H);

            ctx.font = "12px sans-serif";
            ctx.fillStyle = "#0075bf";
            ctx.textAlign = "left";
            ctx.textBaseline = "top";
            ctx.fillText(ep, -EP_W / 2 + 5, -EP_H / 2 + 5);

            ctx.restore();

            x += inc;
        });
    }
}
