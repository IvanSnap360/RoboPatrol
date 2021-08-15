#! /usr/bin/env python3
import rospy
import json
import time
import cv2
import numpy as np
import threading
import requests
import actionlib
import sys
import random
import psutil
from jinja2 import Template
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import leafmap.foliumap as leafmap
from flask import Flask, render_template, Response,request,jsonify
from turbo_flask import Turbo


app = Flask(__name__) 
turbo = Turbo(app)

threading.Thread(target=lambda: rospy.init_node('patrol_app_node', disable_signals=True)).start()

goal_pub = rospy.Publisher("/gps_goal_fix",NavSatFix,queue_size=10)
goals_list = []

start_coords = (56.149568, 40.376083)
m = leafmap.Map(
                google_map="SATELLITE",
                center=start_coords, 
                zoom=30,
                widescreen=True,
                latlon_control=True,
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
            
            
        {% endmacro %}
        """)

draw_control._template = _template
draw_control.export = False
draw_control.add_to(m)

# bm = leafmap.plugins.BoatMarker(start_coords,icon="static/images/arrow.png")
# bm.add_to(m)

sat_count = 0
target_lat = 0
target_long = 0
current_lat = 0
current_long = 0
d_lat = 0
d_long = 0
sys_load = 0
temper = 0
bat = 0
time_left = 0

@app.route('/')
def index(): 
    global sat_count
    global target_lat
    global target_long
    global current_lat
    global current_long   
    
    # print("INDEX_HANDLE")
    #  (56.149568, 40.376083)
    sat_count = 0
    target_lat = 'Now target latitude'
    target_long = 'Now target longtitude'
    current_lat = 'waiting gps...'
    current_lat = 56.149568
    current_long = 'waiting gps...'
    current_long = 40.376083
    goals_list.clear()

    m.to_html('templates/map.html')
    return render_template("index.html")

@app.route("/map")
def map(): 
    # print("MAP_HANDLE")
    return render_template("map.html")

@app.route('/export_data', methods = ['POST'])
def get_data():
    global goals_list
    jsdata = request.form['export_data']
    print(jsdata)
    if jsdata:
        parsed_json = json.loads(jsdata)
        goals_list = parsed_json['geometry']['coordinates']
    else:
        rospy.logwarn("No route")
    return jsdata

@app.route('/start_btn', methods = ['POST'])
def start_btn_cb():
    global target_lat
    global target_long
    for point in goals_list:
        print(point)
        goal = NavSatFix()
        goal.latitude = point[1]
        goal.longitude = point[0]
        target_lat = point[1]
        target_long = point[0]
        goal_pub.publish(goal)
        
        time.sleep(1)
    target_lat = 'Now target latitude'
    target_long = 'Now target longtitude'
    return("nothing")

@app.route('/video_stream')
def video_stream():
    return render_template('video_stream.html')

@app.route('/stop_btn', methods = ['POST'])
def stop_btn_cb():
    return("nothing")

@app.route('/pause_btn', methods = ['POST'])
def pause_btn_cb():
    return("nothing")

@app.route("/location_cb", methods = ['POST'])
def location_cb():
    if type(current_lat) != str or type(current_long) != str: 
        params = {'lat':current_lat,'lon':current_long}
    else:
        params = {'lat':0,'lon':0}
        
    return jsonify(params)

last_time = 0 

@app.context_processor
def inject_load():
    global last_time
    global sat_count
    global target_lat
    global target_long
    global current_lat
    global current_long
    global d_lat
    global d_long
    global temper
    global sys_load
    global bat
    global time_left
    if time.time() - last_time > 1:
        sys_load = psutil.cpu_percent()
        # temper = psutil.sensors_temperatures()['coretemp']
        # bat = psutil.sensors_battery()[0]
        # bat = round(bat,2)
        # time_left = psutil.sensors_battery()[1] / 60
        # time_left = round(time_left,1)
        last_time = time.time()
    return {
            'sat_count': sat_count,
            'targ_lat':target_lat,
            'targ_long':target_long,
            'curr_lat' : current_lat,
            'curr_long':current_long,
            'd_lat':d_lat,
            'd_long':d_long,
            'tempre':temper,
            'lsys_load':sys_load,
            'bat':bat,
            'time_left':time_left
            }

def update_load():
    with app.app_context():
        while True:
            time.sleep(0.01)
            turbo.push(turbo.update(render_template('data.html'), 'load'))

@app.before_first_request
def before_first_request():
    threading.Thread(target=update_load).start()


app.run(host='0.0.0.0', port=8080, debug=True)  

    
