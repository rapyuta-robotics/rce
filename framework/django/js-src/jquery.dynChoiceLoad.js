(function( $ ) {
    $.fn.dynChoiceLoad = function(opts) {
        var options = $.extend({}, $.fn.dynChoiceLoad.defaults, opts), base = this;
        base.change(function() {
            var srvList = $(options.target)[0];
            for (i = srvList.options.length-1; i >= 0; i--) {
                srvList.options[i] = null;
            }
            srvList.disabled = true;
            var pkg = base.val();
            if (pkg.length > 0) {
                var valid = true, addVals = new Object();
                addVals[options.sourceKey] = pkg;
                for (i = 0; i < options.addSources.length; i++) {
                    var tmp = $(options.addSources[i][0]).val();
                    if (valid && tmp.length > 0 && tmp != "undefined") {
                        addVals[options.addSources[i][1]] = tmp;
                    }
                    else {
                        valid = false;
                    }
                }
                if (valid) {
                    $.ajax({
                        type: "GET",
                        url: options.uri,
                        data: addVals,
                        success: function(msg) {
                            var srv = msg.split("\n");
                            for (i=0; i < srv.length; i++) {
                                var parts = srv[i].split("\t");
                                $(options.target).append( new Option(parts[0], parts[1], false, false) )
                            }
                            srvList.disabled = false;
                        },
                        error: function() {
                            srvList.disabled = true;
                        }
                    });
                }
            }
            else {
                srvList.disabled = true;
            }
        });
    };
    
    $.fn.dynChoiceLoad.defaults = {
        uri: "/",                       // URI to where the GET request is sent
        target: "",                     // Selector string for target field which should be updated
        sourceKey: "",                  // Key which is used to identify source field value in GET request
        addSources: []                  // List of Selector, Key pairs which should be used to
    };                                  //   send additional values with the GET request, e.g. [["",""],["",""]]
})( jQuery );
