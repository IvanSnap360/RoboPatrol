#! /usr/bin/env python3
import rospy
import threading
from jinja2 import Template
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import leafmap.foliumap as leafmap
from flask import Flask, render_template, Response,request,jsonify
app = Flask(__name__) 

threading.Thread(target=lambda: rospy.init_node('patrol_app_node', disable_signals=True)).start()

goal_pub = rospy.Publisher("/gps_goal_fix",NavSatFix,queue_size=10)


start_coords = [56.149568, 40.376083]
m = leafmap.Map(
                google_map="SATELLITE",
                center=start_coords, 
                zoom=15,
                widescreen=True,
                latlon_control=False,
                draw_control = False)

draw_control = leafmap.plugins.Draw(position="topleft")

_template = Template(u"""
        {% macro script(this, kwargs) %}
            var options = {
              position: {{ this.position|tojson }},
              draw: {{ this.draw_options|tojson }},
              edit: {{ this.edit_options|tojson }},
            }
            // FeatureGroup is to store editable layers.
            var drawnItems = new L.featureGroup().addTo(
                {{ this._parent.get_name() }}
            );
            options.edit.featureGroup = drawnItems;
            var {{ this.get_name() }} = new L.Control.Draw(
                options
            ).addTo( {{this._parent.get_name()}} );
            {{ this._parent.get_name() }}.on(L.Draw.Event.CREATED, function(e) {
                var layer = e.layer,
                    type = e.layerType;
                var coords = JSON.stringify(layer.toGeoJSON());
                localStorage.setItem("drawn", coords);
                layer.on('click', function() {
                    //alert(coords);
                    console.log(coords);
                    drawnItems.addLayer(layer);
                    var data = drawnItems.toGeoJSON();
                    console.log(coords);
                });
                
             });
            {{ this._parent.get_name() }}.on('draw:created', function(e) {
                drawnItems.addLayer(e.layer);
            });
            {% if this.export %}
            document.getElementById('export').onclick = function(e) {
                var data = drawnItems.toGeoJSON();
                var convertedData = 'text/json;charset=utf-8,'
                    + encodeURIComponent(JSON.stringify(data));
                document.getElementById('export').setAttribute(
                    'href', 'data:' + convertedData
                );
                document.getElementById('export').setAttribute(
                    'download', {{ this.filename|tojson }}
                );
            }
            {% endif %}
            
        {% endmacro %}
        """)

draw_control._template = _template
draw_control.export = False
draw_control.add_to(m)

@app.route('/')
def index():    
    m.to_html('templates/map.html')
    # print("INDEX_HANDLE")
    return render_template("index.html")

@app.route("/map")
def map(): 
    # print("MAP_HANDLE")
    return render_template("map.html")

@app.route('/export_data', methods = ['POST'])
def get_data():
    jsdata = request.form['export_data']
    print(jsdata)
    return jsdata

def start_btn_cb():
    pass

def stop_btn_cb():
    pass

def pause_btn_cb():
    pass

app.run(host='0.0.0.0', port=8080, debug=True)  

    
