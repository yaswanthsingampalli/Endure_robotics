let map, ros;
let destMarker = null;
let currentMarker = null;
let waypointMarkers = [];
let routeLine = null;

let destinationLatLng = null;
let waypointList = [];

let mode = null;

window.initMap = function () {
  // Initialize Google Map
  map = new google.maps.Map(document.getElementById('map'), {
    center: { lat: 37.7777713, lng: -97.1132098 },
    zoom: 15,
  });

  // Connect to rosbridge
  ros = new ROSLIB.Ros({
    url: CONFIG.ROSBRIDGE_SERVER_URL
  });

  ros.on('connection', () => {
    console.log('✅ Connected to rosbridge websocket server.');
  });

  ros.on('error', (error) => {
    console.error('❌ Error connecting to rosbridge websocket server:', error);
  });

  ros.on('close', () => {
    console.log('🔌 Connection to rosbridge websocket server closed.');
  });

  // Map click handler
  map.addListener('click', (event) => {
    if (mode === 'add_waypoint') {
      addWaypoint(event.latLng);
    } else if (mode === 'set_destination') {
      setDestination(event.latLng);
    }
  });

  // Button handlers
  document.getElementById('btnSetDestination').onclick = () => {
    mode = 'set_destination';
    alert('🧭 Click on the map to set the destination.');
  };

  document.getElementById('btnAddWaypoint').onclick = () => {
    mode = 'add_waypoint';
    alert('📍 Click on the map to add waypoints.');
  };

  document.getElementById('btnClearWaypoints').onclick = () => {
    clearWaypoints();
  };

  document.getElementById('btnSendRoute').onclick = () => {
    sendRoute();
  };
};

function addWaypoint(latLng) {
  const marker = new google.maps.Marker({
    position: latLng,
    map: map,
    icon: {
      path: google.maps.SymbolPath.CIRCLE,
      scale: 6,
      fillColor: '#0000FF',
      fillOpacity: 0.8,
      strokeWeight: 1,
      strokeColor: '#FFFFFF',
    },
    title: `Waypoint ${waypointMarkers.length + 1}`
  });

  waypointMarkers.push(marker);
  waypointList.push(latLng);
  console.log(`➕ Waypoint added: ${latLng.lat()}, ${latLng.lng()}`);
  mode = null;
}

function setDestination(latLng) {
  if (destMarker) {
    destMarker.setMap(null);
  }
  destMarker = new google.maps.Marker({
    position: latLng,
    map: map,
    icon: {
      path: google.maps.SymbolPath.BACKWARD_CLOSED_ARROW,
      scale: 8,
      fillColor: '#FF0000',
      fillOpacity: 0.9,
      strokeWeight: 1,
      strokeColor: '#000000',
    },
    title: 'Destination'
  });

  destinationLatLng = latLng;
  console.log(`🎯 Destination set: ${latLng.lat()}, ${latLng.lng()}`);
  mode = null;
}

function clearWaypoints() {
  waypointMarkers.forEach(marker => marker.setMap(null));
  waypointMarkers = [];
  waypointList = [];

  if (destMarker) {
    destMarker.setMap(null);
    destMarker = null;
  }
  destinationLatLng = null;

  if (routeLine) {
    routeLine.setMap(null);
    routeLine = null;
  }

  alert('🧹 Cleared all waypoints and destination. Please select again.');
  console.log('🧹 Map and data cleared.');
}

function drawRouteLine(points) {
  if (routeLine) {
    routeLine.setMap(null);
  }

  routeLine = new google.maps.Polyline({
    path: points,
    geodesic: true,
    strokeColor: '#FF8800',
    strokeOpacity: 0.8,
    strokeWeight: 4,
    map: map
  });
}

function sendRoute() {
  if (!destinationLatLng) {
    alert('❌ Please set a destination first.');
    return;
  }

  if (waypointList.length === 0) {
    alert('❌ Please add at least one waypoint.');
    return;
  }

  const fullRoute = waypointList.concat(destinationLatLng);
  drawRouteLine(fullRoute);

  const routeWaypoints = waypointList.map(p => ({
    x: p.lng(),
    y: p.lat(),
    z: 0.0
  }));

  routeWaypoints.push({
    x: destinationLatLng.lng(),
    y: destinationLatLng.lat(),
    z: 0.0
  });

  // Debug logs
  console.log('📦 Sending route with waypoints:', routeWaypoints);

  callService(CONFIG.ROUTE_SERVICE, { waypoints: routeWaypoints }, 'airport_navigation_interfaces/srv/SetRoute');
}

function callService(serviceName, args, serviceType) {
  const service = new ROSLIB.Service({
    ros: ros,
    name: serviceName,
    serviceType: serviceType
  });

  const request = new ROSLIB.ServiceRequest(args);

  service.callService(request, (result) => {
    if (result.success) {
      alert('✅ Route sent: ' + result.message);
      console.log('✅ ROS Service success:', result.message);
    } else {
      alert('❌ Route send failed: ' + result.message);
      console.warn('❌ ROS Service failure:', result.message);
    }
  }, (error) => {
    alert('❌ Service call error: ' + error);
    console.error('❌ ROS Service error:', error);
  });
}
