let map, ros;
let destMarker = null;
let currentMarker = null;
let waypointMarkers = [];
let routeLine = null;

let destinationLatLng = null;
let waypointList = [];

let mode = null;

window.initMap = function () {
  map = new google.maps.Map(document.getElementById('map'), {
    center: { lat: 37.7777713, lng: -97.1132098 },
    zoom: 15,
  });

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

  map.addListener('click', (event) => {
    if (mode === 'add_waypoint') {
      addWaypoint(event.latLng);
    } else if (mode === 'set_destination') {
      setDestination(event.latLng);
    }
  });

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

  // Initialize the search box on map load
  initSearchBox();
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

  // Reverse geocode the latLng to get a human-readable place name
  const geocoder = new google.maps.Geocoder();
  geocoder.geocode({ location: latLng }, (results, status) => {
    if (status === 'OK' && results[0]) {
      const locationName = results[0].formatted_address;
      console.log(`🎯 Destination set: ${latLng.lat()}, ${latLng.lng()} — ${locationName}`);
      alert(`📍 Destination: ${locationName}`);
    } else {
      console.warn('⚠️ Geocoder failed due to:', status);
      console.log(`🎯 Destination set: ${latLng.lat()}, ${latLng.lng()}`);
    }
  });

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

  const destinationRequest = {
    latitude: destinationLatLng.lat(),
    longitude: destinationLatLng.lng(),
    altitude: 0.0
  };
  console.log('📤 Sending destination:', destinationRequest);
  callService(CONFIG.DEST_SERVICE, destinationRequest, 'airport_navigation_interfaces/srv/SetDestination');

  const routeWaypoints = waypointList.map((p, idx) => ({
    latitude: p.lat(),
    longitude: p.lng(),
    altitude: 0.0,
    name: `Waypoint ${idx + 1}`
  }));
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
      alert('✅ Success: ' + result.message);
      console.log('✅ ROS Service success:', result.message);
    } else {
      alert('❌ Failed: ' + result.message);
      console.warn('❌ ROS Service failure:', result.message);
    }
  }, (error) => {
    alert('❌ Service call error: ' + error);
    console.error('❌ ROS Service error:', error);
  });
}
