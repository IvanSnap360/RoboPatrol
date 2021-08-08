function getKMLfile() {

    var drawnItems = localStorage.getItem("drawn");
    console.debug(drawnItems)
    console.debug(typeof(drawnItems))
    var kml = GeoConvert.geojson2Kml(drawnItems, true);
    console.debug(kml)
    console.debug(typeof(kml))
    download("data.kml", kml)
}