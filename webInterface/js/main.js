var container, camera, controls, scene, renderer;
var FLOOR_SIZE = 1000;
var FONT_SIZE = 60;
var FONT_HEIGHT = 8;
var MAX_HISTORIC_LOC = 300;
var VISIBLE_WAYPOINT_COLOR = 0x0000ff;
var HIDDEN_WAYPOINT_COLOR = 0xccccff;
var PROJECTED_BEACON_COLOR = 0x00ff00;
var FPS = 24;
var PIXELS_X = 1280; // 1920; // 1280;
var PIXELS_Y = 960;  // 1440; // 960;
var HISTORIC_SIZE = 600;
var RECORDING_ENABLED = true;
var PLAYER_ENABLED = false;

var roll = 0.0;
var pitch = 0.0;
var droneRoll = 0.0;
var dronePitch = 0.0;
var droneYaw = 0.0;
var servoRoll = 0.0;
var servoPitch = 0.0;
var heading = 0.0;
var numWaypoints = 0;
var x = 0.0;
var y = 0.0;
var z = 0.1;
var speed = 0.0;
var groundSpeed = 0.0;
var xspeed = 0.0;
var yspeed = 0.0;
var zspeed = 0.0;
var loc_p = 0;
var noloc_p = 0;
var lost = true;
var connected = false;
var ws = null;
var yaw_offset = 0.0;
var piCamera;
var drone;
var historic = [];
var showTrack = true;
var track = null;
var waypoints = {}; // Nominal waypoints
var pixiWidth;
var pixiHeight;
var pixiVmargin = 6;
var pixiStage;
var pixiRenderer;
var pixiDivider;
var pixiGraphics = new PIXI.Graphics();
var pixiTexts = new PIXI.Graphics();
var lastRender = 0;
var beaconData = null;
var waypointData = null;
var speedReadings = 0;
var historicData = [];
var plot;
var dataShown = false;
var dataRecording = false;
var dataRecorded = [];
var dataIndex = 0;
var nextPointIndex = 1;
var navigating = false;

var TAB_3D = 1;
var TAB_CAMERA = 2;
var TAB_DEBUG = 3;
var TAB_CONFIG = 4;
var selectedTab = TAB_3D;

var STATUS_DISCONNECTED = 1;
var STATUS_CONNECTED = 2;
var STATUS_POSITIONED = 3;
var lastStatus = 0;

var n = 1;
var ROLL_VAR = 0x01;
var PITCH_VAR = 0x01 << n++;
var YAW_VAR = 0x01 << n++;
var DRONE_ROLL_VAR = 0x01 << n++;
var DRONE_PITCH_VAR = 0x01 << n++;
var SERVO_ROLL_VAR = 0x01 << n++;
var SERVO_PITCH_VAR = 0x01 << n++;
var WAYPOINTS_VAR = 0x01 << n++;
var X_VAR = 0x01 << n++;
var Y_VAR = 0x01 << n++;
var Z_VAR = 0x01 << n++;
var SPEED_VAR = 0x01 << n++;
var GS_VAR = 0x01 << n++;
var XSPEED_VAR = 0x01 << n++;
var YSPEED_VAR = 0x01 << n++;
var ZSPEED_VAR = 0x01 << n++;
var NUM_VARS = 16;
var GRAPH_VARS = 0;

var varLabels = ["roll", "pitch", "droneRoll", "dronePitch",
                "servoRoll", "servoPitch", "hdg",
                "waypoints", "x", "y", "z", "speed", "gspeed",
                "xspeed", "yspeed", "zspeed"];
var varAxes = [0, 0, 0, 0, 0, 0, 1, 2, 3, 3, 4, 5, 5, 5, 5, 5];
var axesVars = [[0, 1, 2, 3, 4, 5], [6], [7], [8, 9], [10], [11, 12, 13, 14, 15]];
var axesConf = [
    { position: "left", min: -60, max: 60 },
    { position: "left", min: 0, max: 360 },
    { position: "left", min: 0, max: 4 },
    { position: "left", min: -0.5, max: 0.5 },
    { position: "left", min: 0, max: 2 },
    { position: "left", min: -0.5, max: 0.5 }
];
var contentWidth;
var contentHeight;

$(function(){
    init();
    animate();
});

function download() {
    var element = document.createElement('a');
    element.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(JSON.stringify(dataRecorded)));
    element.setAttribute('download', 'data.txt');

    element.style.display = 'none';
    document.body.appendChild(element);

    element.click();

    document.body.removeChild(element);
}

// Output [0 -> 360)
function correctAngle(angle) {
    if (angle >= 360)
        angle %= 360;
    else if (angle <= -360) {
        angle %= 360;
        angle += 360; 
    }
    else if (angle < 0)
        angle += 360; 
    
    return angle;
}

function selectElementContents(el) {
    var body = document.body, range, sel;
    if (document.createRange && window.getSelection) {
        range = document.createRange();
        sel = window.getSelection();
        sel.removeAllRanges();
        try {
            range.selectNodeContents(el);
            sel.addRange(range);
            range.execCommand("Copy");
        } catch (e) {
            range.selectNode(el);
            sel.addRange(range);
            range.execCommand("Copy");
        }
    } else if (body.createTextRange) {
        range = body.createTextRange();
        range.moveToElementText(el);
        range.select();
        range.execCommand("Copy");
    }
}

function integer2string(ival) {
    var x = ival;
    if (ival >= 0) {
        if (ival < 10)
            x = "000" + x;
        else if (ival < 100)
            x = "00" + x;
        else
            x = "0" + x;
    }
    else {
        x = -x;
        if (ival > -10)
            x = "-00" + x;
        else if (ival > -100)
            x = "-0" + x;
        else
            x = "-" + x;
    }
    return x;
}

function setDesiredPosition(x, y) {
    ws.send("t"+ integer2string(x) + integer2string(y));
    //alert(x + ", " + y + " -> " + integer2string(x) + integer2string(y));
}

function navigate() {
    if (!navigating)
        return;
    
    var x = parseInt($(".navCoor[coor='x'][point='" + nextPointIndex + "']").val());
    var y = parseInt($(".navCoor[coor='y'][point='" + nextPointIndex + "']").val());
    var milliseconds = parseInt($("#navSeconds").val()) * 1000;
    nextPointIndex = nextPointIndex % 4 + 1;
    setDesiredPosition(x, y);
    setTimeout("navigate()", milliseconds);
}

function init() {
    window.addEventListener( 'resize', onWindowResize, false );
    resizeLayout();   
    
    // TABS
    $("a#select3d").click(function() {
        selectedTab = TAB_3D;
        $("div#debug").hide();
        $("div#camera").hide();
        $("div#3dViewport").show();
        $("div#config").hide();
        render(true);
    });
    
    $("a#selectCamera").click(function() {
        selectedTab = TAB_CAMERA;
        $("div#debug").hide();
        $("div#3dViewport").hide();
        $("div#camera").show();
        $("div#config").hide();
    });
    
    $("a#selectDebug").click(function() {
        selectedTab = TAB_DEBUG;
        $("div#camera").hide();
        $("div#3dViewport").hide();
        $("div#debug").show();
        $("div#config").hide();
    });
    
    $("a#selectConfig").click(function() {
        selectedTab = TAB_CONFIG;
        $("div#camera").hide();
        $("div#3dViewport").hide();
        $("div#debug").hide();
        $("div#config").show();
        if (PLAYER_ENABLED)
            alert("In offline mode this tab is disabled.");
    });
    
    
    /*****************
     *** DEBUG TAB ***
     *****************/ 
    $("table#varTable input").click(function() {
        var checked = $(this).is(":checked");
        var varMask = 0x01 << parseInt($(this).attr("var"));
        
        if (checked)
            GRAPH_VARS |= varMask;
        else
            GRAPH_VARS &= ~varMask;
        updateYaxes();
    });
    
    // PLOT
    resizePlotWrapper();
    plot = $.plot("#graph", getData(), { 
        series: { shadowSize: 0 },
        yaxes: getYaxes()
    });
    GRAPH_VARS = ROLL_VAR | PITCH_VAR;
    updateYaxes();
    
    /******************
     *** CONFIG TAB ***
     ******************/ 
    if (!PLAYER_ENABLED) {
        $("button#showData").click(function() {
            dataShown = !dataShown;
            
            if (dataShown) {
                $("div#recordedDataContainer").show();
                //dataRecording = true;
            }
            else
                $("div#recordedDataContainer").hide();
        });
        
        $("div#recordedDataContainer").width(contentWidth - 265)
            .height(contentHeight - 115);
        
        $("button#selectData").click(function() {
            selectElementContents(document.getElementById("recordedData"));
        });
        
        $("button#clearData").click(function() {
            $("div#recordedData").html("");
        });
        
        $("button#stopRecordingFile").click(function() {
            dataRecording = false;
        });
        
        $("button#startRecordingFile").click(function() {
            dataRecording = true;
        });
        
        $("button#enableServos").click(function() {
            ws.send("e");
        });

        $("button#disableServos").click(function() {
            ws.send("d");
        });
        
        $( "input#servoSpeed" ).on( 'slidestop', function( event ) { 
            var val = $(this).val();
            if (val < 10)
                val = "0" + val;
            ws.send("s"+ val);
        });
        
        $( "input#maxServoRollSpeed" ).on( 'slidestop', function( event ) { 
            var val = $(this).val();
            if (val < 10)
                val = "0" + val;
            ws.send("m"+ val);
        });
        
        $( "input#maxServoPitchSpeed" ).on( 'slidestop', function( event ) { 
            var val = $(this).val();
            if (val < 10)
                val = "0" + val;
            ws.send("n"+ val);
        });
        
        $( "input#attitudeDelay" ).on( 'slidestop', function( event ) { 
            var val = $(this).val();
            if (val < 10)
                val = "0" + val;
            ws.send("a"+ val);
        });
        
        $( "input#rollServoDelay" ).on( 'slidestop', function( event ) { 
            var val = $(this).val();
            if (val < 10)
                val = "0" + val;
            ws.send("y"+ val);
        });
        
        $( "input#pitchServoDelay" ).on( 'slidestop', function( event ) { 
            var val = $(this).val();
            if (val < 10)
                val = "0" + val;
            ws.send("z"+ val);
        });
        
        $( "input#rollServoOffset" ).on( 'change', function( event ) { 
            var val = parseInt($(this).val()) + 499;
            if (val < 100) val = "0" + val;
            if (val < 10) val = "0" + val;
            ws.send("o"+ val);
        });
        
        $( "input#pitchServoOffset" ).on( 'change', function( event ) { 
            var val = parseInt($(this).val()) + 499;
            if (val < 100) val = "0" + val;
            if (val < 10) val = "0" + val;
            ws.send("p"+ val);
        });
        
        $( "input#rollServoRange" ).on( 'slidestop', function( event ) { 
            var val = $(this).val();
            if (val < 10)
                val = "0" + val;
            ws.send("v"+ val);
        });
        
        $( "input#pitchServoRange" ).on( 'slidestop', function( event ) { 
            var val = $(this).val();
            if (val < 10)
                val = "0" + val;
            ws.send("w"+ val);
        });
        
        $( "#applyPosition" ).click( function() {
            setDesiredPosition(parseInt($("#actualX").val()), parseInt($("#actualY").val()));
        });
        
        $( "#startNavigation" ).click( function() {
            nextPointIndex = 1;
            navigating = true;
            navigate();
        });
        
        $( "#stopNavigation" ).click( function() {
            navigating = false;
        });
        
        if(RECORDING_ENABLED)
            $("div#recordDiv").show();
    }
    
    
    /******************
     *** CAMERA TAB ***
     ******************/ 
    resizePixi(false);
    pixiStage = new PIXI.Stage(0x000000);
    pixiStage.alpha = 50;
    pixiStage.addChild(pixiGraphics);
    pixiStage.addChild(pixiTexts);
    pixiRenderer = PIXI.autoDetectRenderer(pixiWidth, pixiHeight);
    $("div#pixi").css("left", parseInt((contentWidth - pixiWidth) / 2) + "px");
    $("div#pixi").html(pixiRenderer.view);
 
    
    /**************
     *** 3D TAB ***
     **************/ 
    camera = new THREE.PerspectiveCamera( 60, contentWidth / contentHeight,
                                          1, 10000 );
    camera.position.z = 800;
    camera.position.y = -600;
    controls = new THREE.OrbitControls( camera, 
                                        document.getElementById("3dViewport") );
    controls.addEventListener( 'change', render );
    scene = new THREE.Scene();

    // lights
    light = new THREE.DirectionalLight( 0xffffff );
    light.position.set( 0, -0.5, 1 );
    scene.add( light );
    light = new THREE.AmbientLight( 0x333333 );
    scene.add( light );

    // renderer
    renderer = new THREE.WebGLRenderer( { antialias: false } );
    renderer.setClearColor( 0xffffff, 1 );
    renderer.setSize( contentWidth, contentHeight );
    container = document.getElementById( '3dViewport' );
    container.appendChild( renderer.domElement );
    
    // Floor
    var geometry = new THREE.PlaneGeometry( FLOOR_SIZE*2, FLOOR_SIZE*2, 1, 1 );
    var material = new THREE.MeshBasicMaterial( { color: 0xdddddd } );
    var floor = new THREE.Mesh( geometry, material );
    floor.position.z = 0;
    floor.material.side = THREE.DoubleSide;
    scene.add( floor );
    
    // X Axis
    material = new THREE.LineBasicMaterial( {
        linewidth: 1,
        color: '#ff9999'
    });
    geometry = new THREE.Geometry();
    geometry.vertices.push( new THREE.Vector3( -FLOOR_SIZE, 0, 1 ), 
                            new THREE.Vector3( FLOOR_SIZE, 0, 1 ));
    scene.add(new THREE.Line( geometry, material, THREE.LinePieces ));
    
    geometry = new THREE.CylinderGeometry( 0, 5, 25, 15, 1 );
    material = new THREE.MeshLambertMaterial( { 
        color:0xff9999,
        shading: THREE.FlatShading 
    } );
    mesh = new THREE.Mesh( geometry, material );
    mesh.position.x = FLOOR_SIZE;
    mesh.position.y = 0;
    mesh.position.z = 0;
    mesh.rotation.z = -Math.PI / 2;
    scene.add( mesh );
    
    geometry = new THREE.TextGeometry( "X", {
        size: FONT_SIZE,
        height: FONT_HEIGHT 
    } );
    var mesh = new THREE.Mesh( geometry, material );
    mesh.position.x = FLOOR_SIZE + 30;
    mesh.position.y = -7;
    mesh.position.z = 0;
    scene.add( mesh );
    
    // Y Axis
    material = new THREE.LineBasicMaterial({ linewidth: 1, color: '#99ff99' });
    geometry = new THREE.Geometry();
    geometry.vertices.push( new THREE.Vector3( 0, -FLOOR_SIZE, 1 ), 
                            new THREE.Vector3( 0, FLOOR_SIZE, 1 ));
    scene.add(new THREE.Line( geometry, material, THREE.LinePieces ));
    
    geometry = new THREE.CylinderGeometry( 0, 5, 25, 15, 1 );
    material = new THREE.MeshLambertMaterial( { 
        color:0x99ff99,
        shading: THREE.FlatShading
    } );
    mesh = new THREE.Mesh( geometry, material );
    mesh.position.x = 0;
    mesh.position.y = FLOOR_SIZE;
    mesh.position.z = 0;
    scene.add( mesh );
    
    geometry = new THREE.TextGeometry( "Y", {
        size: FONT_SIZE,
        height: FONT_HEIGHT 
    } );
    var mesh = new THREE.Mesh( geometry, material );
    mesh.position.x = -2;
    mesh.position.y = FLOOR_SIZE + 30;
    mesh.position.z = 0;
    scene.add( mesh );
    
    // Z Axis
    material = new THREE.LineBasicMaterial({
        linewidth: 1,
        color: '#9999ff'
    });
    geometry = new THREE.Geometry();
    geometry.vertices.push( new THREE.Vector3( 0, 0, -FLOOR_SIZE ),  
                            new THREE.Vector3( 0, 0, FLOOR_SIZE ));
    scene.add(new THREE.Line( geometry, material, THREE.LinePieces ));
    
    geometry = new THREE.CylinderGeometry( 0, 5, 25, 15, 1 );
    material = new THREE.MeshLambertMaterial( { 
        color:0x9999ff,
        shading: THREE.FlatShading
    } );
    mesh = new THREE.Mesh( geometry, material );
    mesh.position.x = 0;
    mesh.position.y = 0;
    mesh.position.z = FLOOR_SIZE;
    mesh.rotation.x = Math.PI / 2;
    scene.add( mesh );
    
    geometry = new THREE.TextGeometry( "Z", {
        size: FONT_SIZE,
        height: FONT_HEIGHT
    } );
    var mesh = new THREE.Mesh( geometry, material );
    mesh.position.x = 0;
    mesh.position.y = 0;
    mesh.position.z = FLOOR_SIZE + 30;
    mesh.rotation.x = Math.PI / 2;
    scene.add( mesh );
    
    // Drone
    initDrone();
    
    // Load Nominal Waypoints
    $.get("waypoints.xml", function(data) {
        $("waypoint", data).each(function() {
            var tx = 0.0;
            var ty = 0.0;
            var tz = 0.0;
            var master = false;
            
            if (typeof $(this).attr("x") !== typeof undefined)
                tx = parseFloat($(this).attr("x"));
            
            if (typeof $(this).attr("y") !== typeof undefined)
                ty = parseFloat($(this).attr("y"));
            
            if (typeof $(this).attr("z") !== typeof undefined)
                tz = parseFloat($(this).attr("z"));
            
            if (typeof $(this).attr("master") !== typeof undefined)
                master = parseInt($(this).attr("master"));
            
            var id = $(this).attr("id");
            var name = $(this).attr("name");
            
            waypoints[id] = {};
            
            $("beacon", this).each(function() {
                var xb = parseFloat($(this).attr("x"));
                var yb = parseFloat($(this).attr("y"));
                var zb = parseFloat($(this).attr("z"));
                var num = parseInt($(this).attr("num"));                
                
                var cone = createCone(tx+ xb,
                                      ty + yb,
                                      tz + zb,
                                      HIDDEN_WAYPOINT_COLOR,
                                      1);
                scene.add(cone);
                
                waypoints[id][num] = cone;
            });
        });        
        render();
    });
   
    render();
    
    if (PLAYER_ENABLED) {
        $("#headerTitle").html("Downloading data, please wait...");
        $.getJSON("data.json", function( data ) {
            $("#headerTitle").html("Infrared beacon positioning system");
            dataRecorded = data;
            $("div#statusIcon").removeClass("disconnected")
                .removeClass("locked").addClass("connected");
            
            setInterval(function() { playStep() }, 25);
        }).fail(function() {
            alert("Check your network connection. Error: Unable to download data.json file.");
        });
    }
    else {
        if ("WebSocket" in window)
            tryConnect();
        else 
            alert("WebSocket NOT supported by your Browser!");
    }
}


/****************
 **** PLAYER ****
 ****************/
function playStep() {
    if (dataIndex >= dataRecorded.length)
        dataIndex = 0;
    
    processDataUnit($.parseJSON(dataRecorded[dataIndex++]));
}


/*****************
 **** NETWORK ****
 *****************/
function processDataUnit(data) {
    switch (data.type) {
    case "ph":
        if (lastStatus != STATUS_POSITIONED) {
            $("div#statusIcon").removeClass("disconnected")
                .removeClass("connected").addClass("positioned");
        }
            
        x = data.x;
        y = data.y;
        z = data.z;
        numWaypoints = data.numwaypoints;
        heading = data.heading;
        speed = data.speed;
        xspeed = data.xspeed;
        yspeed = data.yspeed;
        zspeed = data.zspeed;
        groundSpeed = data.groundspeed;
        yaw_offset = correctAngle(heading - droneYaw);
        roll = data.cameraRoll;
        pitch = data.cameraPitch;
        droneRoll = data.droneRoll;
        dronePitch = data.dronePitch;
        servoRoll = data.servoRoll;
        servoPitch = data.servoPitch;
        beaconData = data.beacons;
        waypointData = data.waypoints;
        loc_p++;
        
        if (selectedTab == TAB_DEBUG) {
            addHistoryData(roll, pitch, droneRoll, dronePitch,
                            servoRoll, servoPitch, heading, 
                            numWaypoints, x, y, z, speed, groundSpeed,
                            xspeed, yspeed, zspeed);
        }
        
        var position = new THREE.Vector3(x, y, z);
        
        /*if (speed > 1) {
            if (++speedReadings > 15) {
                $("div#log").html("");
                speedReadings = 0;
            }
            $("div#log").append("Speed: " + speed.toFixed(3)
                + " m/s, photogram: " + data.photogram  + "<br>");
        }*/
        
        if (showTrack) {
            historic.push(new THREE.Vector3(x * 1000, y * 1000, z * 1000));
            if (historic.length > MAX_HISTORIC_LOC)
                historic.shift();
        }
        
        render();
        lost = false;
        lastStatus = STATUS_POSITIONED;
        break;
        
    case "nf":
        heading = droneYaw + yaw_offset;
        if (lastStatus != STATUS_CONNECTED) {
            $("div#statusIcon").removeClass("disconnected")
                .removeClass("positioned").addClass("connected");
        }
        lost = true;
        noloc_p++;
        numWaypoints = 0;
        roll = data.cameraRoll;
        pitch = data.cameraPitch;
        droneRoll = data.droneRoll;
        dronePitch = data.dronePitch;
        droneYaw = data.droneYaw;
        servoRoll = data.servoRoll;
        servoPitch = data.servoPitch;
        beaconData = data.beacons;
        waypointData = data.waypoints;
        
        if (dataRecording) {
            $("div#recordedData").append("nf;" +
                heading + ";" + 
                roll + ";" + 
                pitch + ";" + 
                droneRoll + ";" + 
                dronePitch + ";" + 
                droneYaw + ";" + 
                servoRoll + ";" + 
                servoPitch + ";" + 
                JSON.stringify(beaconData) + ";" + 
                JSON.stringify(waypointData) + "<br>");
        }
        
        if (selectedTab == TAB_DEBUG) {
            addHistoryData(roll, pitch, droneRoll, dronePitch, servoRoll, servoPitch, heading, 0,
                            null, null, null, null, null, null, null, null);
        }
        
        lastStatus = STATUS_CONNECTED;
        render();
        break;
    }    
}

function tryConnect() {
    if (connected)
        return;
    
    ws = new WebSocket("ws://" + window.location.host.split(":")[0] + 
        ":8080", 'dumb-increment-protocol');
    ws.onopen = function() {
        connected = true;
        $("div#statusIcon").removeClass("disconnected")
            .removeClass("locked").addClass("connected");
    };
    ws.onmessage = function (evt) {                 
        if (dataRecording)
            dataRecorded.push(evt.data);
        
        var data = jQuery.parseJSON( evt.data );
        
       processDataUnit(data);
    };
    ws.onclose = function() {
        connected = false;
        window.setTimeout("tryConnect();", 1000);
        $("div#statusIcon").removeClass("connected")
            .removeClass("positioned").addClass("disconnected");
    };
}


/***************************
 *** 2D CAMERA FUNCTIONS ***
 ***************************/
function updateCamera() {
    pixiTexts.clear();
    pixiTexts.removeChildren();
    
    pixiGraphics.clear();
    pixiGraphics.removeChildren();
    
    if (beaconData != null && waypointData != null) {
        pixiGraphics.lineStyle(1, 0xFF0000);
        // Draw beacons
        for (var beaconId = 0; beaconId < beaconData.length; beaconId++) {
            var id = beaconData[beaconId].id;
            var x = beaconData[beaconId].xi / pixiDivider;
            var y = beaconData[beaconId].yi / pixiDivider;
            var radius = parseInt((Math.sqrt(beaconData[beaconId].size / Math.PI))
                / pixiDivider);
            var discarded = beaconData[beaconId].discarded;
            
            if (radius <= 0)
                size = 1;
            
            pixiGraphics.beginFill(0xFFFFFF, 1);
            pixiGraphics.lineStyle(0);
            pixiGraphics.drawCircle(x, y, radius);
            pixiGraphics.endFill();
            
            var text = new PIXI.Text(id.toString(), {
                font:"14px Arial",
                fill:"green"
            });
            
            var offsetX = 4;
            var offsetY = 8;
            
            if (radius < 8 || discarded) {
                text.position.y = y + radius;
                text.position.x = x - offsetX;
            }
            else {
                text.position.y = y - offsetY;
                text.position.x = x - offsetX;
            }
            pixiTexts.addChild(text);
            
            if (discarded) {
                var length = radius * 2;
                if (length < 10)
                    length = 10;
                
                pixiGraphics.lineStyle(2, 0xFF0000);
                pixiGraphics.moveTo(x - length, y - length);
                pixiGraphics.lineTo(x + length, y + length);
                pixiGraphics.moveTo(x + length, y - length);
                pixiGraphics.lineTo(x - length, y + length);
                pixiGraphics.lineStyle(0);
            }
        }
        
        // Draw beacon groups/waypoints
        for (var beaconGroupId = 0; beaconGroupId < waypointData.length;
            beaconGroupId++) {
            
            var beaconGroup = waypointData[beaconGroupId]
            var positioned = beaconGroup.positioned;
            var beacons = waypointData[beaconGroupId].beacons;
            
            // Geat bounding box
            var minx = Infinity;
            var miny = Infinity;
            var maxx = 0;
            var maxy = 0;
            for (var i = 0; i < beacons.length; i++) {
                var beaconIndex = beacons[i];
                var beacon = beaconData[beaconIndex];
                
                if (beacon == undefined)
                    alert();
                
                var radius = parseInt((Math.sqrt(beacon.size / Math.PI)));
                
                if (beacon.xi + radius > maxx)
                    maxx = beacon.xi + radius;
                if (beacon.yi + radius > maxy)
                    maxy = beacon.yi + radius;
                if (beacon.xi - radius < minx) 
                    minx = beacon.xi - radius;
                if (beacon.yi - radius < miny) 
                    miny = beacon.yi - radius;
            }
            
            minx /= pixiDivider;
            miny /= pixiDivider;
            maxx /= pixiDivider;
            maxy /= pixiDivider;
            
            var width = maxx - minx;
            var height = maxy - miny;
            
            // Draw rectangle
            if (positioned) {
                //pixiGraphics.lineStyle(0);
                var text = new PIXI.Text("Waypoint id: "
                    + beaconGroup.waypoint.toString(),
                    {font:"18px Arial", fill:"blue"});
                
                text.position.x = parseInt(minx);
                text.position.y = parseInt(miny) - 20;
                pixiTexts.addChild(text);
                pixiGraphics.lineStyle(2, 0x0000ff);
            }
            else
                pixiGraphics.lineStyle(1, 0xff0000);
                
            pixiGraphics.drawRect(minx, miny, width, height);
        }
    }
    pixiRenderer.render(pixiStage);
}


/***********************
 *** DEBUG FUNCTIONS ***
 ***********************/
function updateDebug() {
    heading = correctAngle(droneYaw + yaw_offset);
    $("span#roll").html(roll.toFixed(1));
    $("span#pitch").html(pitch.toFixed(1));
    $("span#droneRoll").html(droneRoll.toFixed(1));
    $("span#dronePitch").html(dronePitch.toFixed(1));
    $("span#servoRoll").html(servoRoll.toFixed(1));
    $("span#servoPitch").html(servoPitch.toFixed(1));
    $("span#waypoints").html(numWaypoints);
    $("span#heading").html(heading.toFixed(1));
    
    if (lost) {
        $("span#noloc_p").html(noloc_p);
        $("span#x").html("");
        $("span#y").html("");
        $("span#z").html("");
        $("span#speed").html("");
        $("span#xspeed").html("");
        $("span#yspeed").html("");
        $("span#zspeed").html("");
        $("span#gspeed").html("");
    }
    else {
        $("span#x").html(x.toFixed(3));
        $("span#y").html(y.toFixed(3));
        $("span#z").html(z.toFixed(3));
        $("span#speed").html(speed.toFixed(3));
        $("span#gspeed").html(groundSpeed.toFixed(3));
        $("span#xspeed").html(xspeed.toFixed(3));
        $("span#yspeed").html(yspeed.toFixed(3));
        $("span#zspeed").html(zspeed.toFixed(3));
        $("span#loc_p").html(loc_p);
    }            
    updateGraph();
}
            
function addHistoryData(r, p, dr, dp, sr, sp, h, w, x, y, z, s, gs, xs, ys, zs) {
    if (historicData.length >= HISTORIC_SIZE)
        historicData = historicData.slice(1);
    historicData.push([r, p, dr, dp, sr, sp, h, w, x, y, z, s, gs, xs, ys, zs]);
}

function getData() {
    var data = [];
    for (var i = 0; i < NUM_VARS; i++) {
        var varMask = 0x01 << i;
        if (varMask & GRAPH_VARS) {
            var varSerie = {};
            varSerie.data = get2darray(i);
            varSerie.label = varLabels[i];
            varSerie.yaxis = varAxes[i] + 2;
            data.push(varSerie);
        }
    }
    return data;
}

function get2darray(axis) {    
    var array2d = []
    var i;
    for (i=0; i < historicData.length; i++)
        array2d.push([i, historicData[i][axis]])
        
    for (i = historicData.length; i < HISTORIC_SIZE; i++)
        array2d.push([i, null]);
    
    return array2d;
}

function updateGraph() {
    plot.setData(getData());
    plot.setupGrid()
    plot.draw();
}

function getYaxes() {
    var yaxes = [{ }];
    for (var i = 0; i < axesVars.length; i++) {
        var show = false;
        for (var j = 0; j < axesVars[i].length; j++) {
            var varMask = 0x01 << axesVars[i][j];
            if (varMask & GRAPH_VARS) {
                show = true;
                break;
            }
        }
        
        axesConf[i].show = show;
        yaxes.push(axesConf[i]);
    }
    return yaxes;
}

function updateYaxes() {
    var axes = plot.getAxes();
    
    for (var i = 0; i < axesVars.length; i++) {
        var show = false;
        for (var j = 0; j < axesVars[i].length; j++) {
            var varMask = 0x01 << axesVars[i][j];
            if (varMask & GRAPH_VARS) {
                show = true;
                break;
            }
        }
        
        axes["y" + (i + 2) + "axis"].options.show = show;
    }
}


/********************
 *** 3D FUNCTIONS ***
 ********************/
function removeObject(object) {
    if (object != null) {
        scene.remove( object );
        $.each(object.children, function (idx, obj) {
            object.remove(obj);
            /*if (typeof obj.geometry != "undefined")
                obj.geometry.dispose();
            if (typeof obj.material != "undefined")
                obj.material.dispose();*/
        });
        object = null;
    }
}

function update3d() {
    // Remove previous drawing
    removeObject(piCamera);
    piCamera = new THREE.Object3D();
    
    var height = 90.;
    var hFov = 0.933751149817;
    var vFov = 0.722740843251;
    
    var halfWidth = height * Math.tan( hFov/2 );
    var halfHeight = height * Math.tan( vFov/2 );
    
    var p1 = new THREE.Vector3( halfWidth, halfHeight, -height );
    var p2 = new THREE.Vector3( halfWidth, -halfHeight, -height );
    var p3 = new THREE.Vector3( -halfWidth, -halfHeight, -height );
    var p4 = new THREE.Vector3( -halfWidth, halfHeight, -height );
    
    var pitchVector = new THREE.Vector3( 1, 0, 0 );
    var rollVector = new THREE.Vector3( 0, 1, 0 );
    var yawVector = new THREE.Vector3( 0, 0, -1 );
    
    // HEADING
    var quaternion = new THREE.Quaternion();
    quaternion.setFromAxisAngle( yawVector, heading * Math.PI/180);
    
    pitchVector.applyQuaternion( quaternion );
    rollVector.applyQuaternion( quaternion );
    
    // PITCH
    var quaternionPitch = new THREE.Quaternion();
    quaternionPitch.setFromAxisAngle( pitchVector, pitch * Math.PI/180);
    rollVector.applyQuaternion( quaternionPitch );
    
    // ROLL
    var quaternionRoll = new THREE.Quaternion();
    quaternionRoll.setFromAxisAngle( rollVector, roll * Math.PI/180);
    
    // Get combined quaternion
    quaternion.multiplyQuaternions( quaternionPitch, quaternion );
    quaternion.multiplyQuaternions( quaternionRoll, quaternion );
    
    p1.applyQuaternion( quaternion );
    p2.applyQuaternion( quaternion );
    p3.applyQuaternion( quaternion );
    p4.applyQuaternion( quaternion );
    
    // Origin
    var o = new THREE.Vector3( x * 1000, y * 1000, z * 1000 );
    
    p1.add(o);
    p2.add(o);
    p3.add(o);
    p4.add(o);
    
    // Extension to ground plane
    var floor = new THREE.Plane(new THREE.Vector3(0, 0, 1), -0.5);
    
    var p1f = new THREE.Vector3();
    var p2f = new THREE.Vector3();
    var p3f = new THREE.Vector3();
    var p4f = new THREE.Vector3();
    
    var p1v = new THREE.Vector3();
    var p2v = new THREE.Vector3();
    var p3v = new THREE.Vector3();
    var p4v = new THREE.Vector3();
    
    p1f.copy(p1);
    p2f.copy(p2);
    p3f.copy(p3);
    p4f.copy(p4);
    
    p1v.copy(p1);
    p2v.copy(p2);
    p3v.copy(p3);
    p4v.copy(p4);
    
    p1v.sub( o ).multiplyScalar( 100 );
    p2v.sub( o ).multiplyScalar( 100 );
    p3v.sub( o ).multiplyScalar( 100 );
    p4v.sub( o ).multiplyScalar( 100 );
    
    var l1 = new THREE.Line3(o, p1f.add( p1v ));
    var l2 = new THREE.Line3(o, p2f.add( p2v ));
    var l3 = new THREE.Line3(o, p3f.add( p3v ));
    var l4 = new THREE.Line3(o, p4f.add( p4v ));
    
    var hasIntersection = floor.isIntersectionLine(l1) 
        && floor.isIntersectionLine(l2)
        && floor.isIntersectionLine(l3)
        && floor.isIntersectionLine(l4);

    if (hasIntersection) {
        var p1f = floor.intersectLine(l1);
        var p2f = floor.intersectLine(l2);
        var p3f = floor.intersectLine(l3);
        var p4f = floor.intersectLine(l4);
    }
    
    // Camera viewport
    var color = lost ? 'red' : 'black';
    var material = new THREE.LineBasicMaterial({ color: color });
    var geometry = new THREE.Geometry();
    geometry.vertices.push( o,p1, o,p2, o,p3, o,p4, p1,p2, p2,p3, p3,p4, p4,p1);
    piCamera.add(new THREE.Line( geometry, material, THREE.LinePieces ));
    
    if (hasIntersection) {
        // Campera projection to the ground
        material = new THREE.LineBasicMaterial({ color: '#999999' });
        geometry = new THREE.Geometry();
        geometry.vertices.push( p1,p1f, p2,p2f, p3,p3f, p4,p4f,
                                p1f,p2f, p2f,p3f, p3f,p4f, p4f,p1f );
        piCamera.add(new THREE.Line( geometry, material, THREE.LinePieces ));
    }
    
    // Positioning information
    var nadir = new THREE.Vector3();
    var nadirXproj = new THREE.Vector3();
    var nadirYproj = new THREE.Vector3();
    var zero = new THREE.Vector3();
    
    nadir.x = o.x;
    nadir.y = o.y;
    nadir.z = 1;
    
    nadirXproj.x = o.x;
    nadirXproj.y = 0;
    nadirXproj.z = 1;
    
    nadirYproj.x = 0;
    nadirYproj.y = o.y;
    nadirYproj.z = 1;
    
    zero.z = 1;
    
    geometry = new THREE.Geometry();
    geometry.vertices.push( o,nadir );
    piCamera.add(new THREE.Line( geometry, new THREE.LineDashedMaterial( {
        linewidth: 2,
        color: 0x9999ff,
        dashSize: 16,
        gapSize: 4 
    } ), THREE.LinePieces ));
    geometry.computeLineDistances();
    
    geometry = new THREE.Geometry();
    geometry.vertices.push( nadir, nadirYproj );
    piCamera.add(new THREE.Line( geometry, new THREE.LineDashedMaterial( {
        linewidth: 2,
        color: 0x99ff99,
        dashSize: 16,
        gapSize: 4
    } ), THREE.LinePieces ));
    geometry.computeLineDistances();
    
    geometry = new THREE.Geometry();
    geometry.vertices.push( nadir, nadirXproj );
    piCamera.add(new THREE.Line( geometry, new THREE.LineDashedMaterial( {
        linewidth: 2,
        color: 0xff9999,
        dashSize: 16,
        gapSize: 4
    } ), THREE.LinePieces ));
    geometry.computeLineDistances();
    
    if (beaconData != null) {
        // Draw projected beacons        
        for (var beaconId = 0; beaconId < beaconData.length; beaconId++) {
            var bVector = new THREE.Vector3( beaconData[beaconId].xp * z,
                                             beaconData[beaconId].yp * z, 0);
            bVector.applyAxisAngle(new THREE.Vector3(0, 0, -1),
                                   heading * Math.PI/180);
            var cone = createCone(bVector.x + x, bVector.y + y,
                                  0.01, PROJECTED_BEACON_COLOR, 1.1);
            piCamera.add(cone);
        }
    }
    
    scene.add( piCamera );
    
    if (!lost && showTrack)
      drawTrack();
    
    // Update drone
    pitchVector = new THREE.Vector3( 1, 0, 0 );
    rollVector = new THREE.Vector3( 0, 1, 0 );
    yawVector = new THREE.Vector3( 0, 0, -1 );
    
    // Apply origin
    drone.position.x = o.x;
    drone.position.y = o.y;
    drone.position.z = o.z;
    
    // Initialize rotations
    drone.rotation.x = 0;
    drone.rotation.y = 0;
    drone.rotation.z = 0;
    
    drone.rotateOnAxis( yawVector, heading * Math.PI/180 );
    drone.rotateOnAxis( pitchVector, dronePitch * Math.PI/180 );
    drone.rotateOnAxis( rollVector, droneRoll * Math.PI/180 );
    
    renderer.render( scene, camera );
}

function initDrone() {
    var p1 = new THREE.Vector3( 100, -60, 25 );
    var p2 = new THREE.Vector3( 100, -215, 25 );
    var p3 = new THREE.Vector3( -100, -60, 25 );
    var p4 = new THREE.Vector3( -100, -215, 25 );
    
    var material = new THREE.MeshBasicMaterial({
        color: 0x333333
    });
    
    material.opacity = 0.25;
    material.transparent = true;

    var radius = 62.5;
    var segments = 32;
    
    drone = new THREE.Object3D();
    
    var circleGeometry = new THREE.CircleGeometry( radius, segments );              
    var circle = new THREE.Mesh( circleGeometry, material );
    circle.position = p1;
    drone.add( circle );
    
    var circleGeometry = new THREE.CircleGeometry( radius, segments );              
    var circle = new THREE.Mesh( circleGeometry, material );
    circle.position = p2;
    drone.add( circle );
    
    var circleGeometry = new THREE.CircleGeometry( radius, segments );              
    var circle = new THREE.Mesh( circleGeometry, material );
    circle.position = p3;
    drone.add( circle );
    
    var circleGeometry = new THREE.CircleGeometry( radius, segments );              
    var circle = new THREE.Mesh( circleGeometry, material );
    circle.position = p4;
    drone.add( circle );
    
    scene.add( drone );
}

function drawTrack() {
    removeObject(track);
    track = new THREE.Object3D();
    var geometry = new THREE.Geometry();
    geometry.vertices = historic.slice(0);
    var material = new THREE.LineBasicMaterial({ color: '#999999' });    
    track.add(new THREE.Line( geometry, material, THREE.LineStrip ));
    scene.add(track);
}

function createCone(x, y, z, color, size) {
    var geometry = new THREE.CylinderGeometry( 0, 5 * size, 10 * size, 20, 1 );    
    var material = new THREE.MeshLambertMaterial( {
        color:color,
        shading: THREE.FlatShading 
    } );
    var mesh = new THREE.Mesh( geometry, material );
    mesh.position.x = x * 1000;
    mesh.position.y = y * 1000;
    mesh.position.z = z * 1000 - 5;
    mesh.rotation.x = Math.PI / 2;
    return mesh;
}

function animate() {
    requestAnimationFrame( animate );
    controls.update();
}


/************************
 *** RESIZE FUNCTIONS ***
 ************************/
function onWindowResize() {
    resizeLayout();   
    resize3D();
    resizePixi(true);
    resizePlot();
}

function resizeLayout() {
    contentWidth = $(window).width();
    contentHeight = $(window).height() 
        - $("div[data-role=\"header\"").outerHeight();
    $("div.tab").height(contentHeight)
}

function resizePlotWrapper() {
    $("div#debug").height(contentHeight);
    $("div#graph").width(contentWidth - 324).height(contentHeight - 14);
}

function resizePlot() {
    resizePlotWrapper();
    plot.resize();
    plot.setupGrid();
    plot.draw();
}

function resizePixi(render) {
    var margin = 12;
    
    if (contentWidth/contentHeight > 1.3333) {
        pixiHeight = contentHeight - margin;
        pixiWidth = parseInt((pixiHeight * 4) / 3);
    }
    else {
        pixiWidth = contentWidth - margin;
        pixiHeight = parseInt((pixiWidth * 3) / 4);
    }
    $("div#pixi").css("width", pixiWidth + "px");
    $("div#pixi").css("height", pixiHeight + "px");
    $("div#pixi").css("left", parseInt((contentWidth - pixiWidth) / 2) + "px");
    pixiDivider = PIXELS_X / pixiWidth;
    if (render)
        pixiRenderer.resize(pixiWidth, contentHeight);
}

function resize3D() {
    camera.aspect = contentWidth / contentHeight;
    camera.updateProjectionMatrix();
    renderer.setSize( contentWidth, contentHeight );
    render();
}

function render(force) {
    var d = new Date();
    var now = d.getTime();
    if (now - lastRender >= 1000/FPS
        || (typeof force !== "undefined" && force == true)) {
        
        if (selectedTab == TAB_3D) {
            update3d();
        }
        else if (selectedTab == TAB_CAMERA) {
            updateCamera();
        }
        else if (selectedTab == TAB_DEBUG) {
            updateDebug();
        }
        lastRender = now;
    }
}
