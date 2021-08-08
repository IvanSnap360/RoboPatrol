function getParam() {
    var drawnItems = localStorage.getItem("drawn");
    console.debug(drawnItems)
    console.debug(typeof(drawnItems))
        // $.post('/export', {drawnItems}, function(){})
    $.post("/export_data", {
        export_data: drawnItems
    });
}