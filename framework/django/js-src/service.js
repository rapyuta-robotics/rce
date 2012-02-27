$(document).ready(function() {
    $("#id_pkg").change( function() {
        var srvList = $("#id_srvDef")[0];
        for (i = srvList.options.length-1; i >=0; i--){
            srvList.options[i] = null;
        }
        srvList.disabled = true;
        var pkg = $("#id_pkg").val();
        if (pkg.length > 0) {
            $.ajax({
                type: "GET",
                url: "/service/select/" + pkg + "/",
                success: function(msg) {
                    var srv = msg.split("\n");
                    for (i=0; i<srv.length; i++) {
                        $("#id_srvDef").append( new Option(srv[i], srv[i], false, false) )
                    }
                    srvList.disabled = false;
                },
                error: function() {
                    srvList.disabled = false;
                }
            });
        }
        else {
            srvList.disabled = true;
        }
    });
});
