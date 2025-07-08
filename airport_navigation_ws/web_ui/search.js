// search.js

function initSearchBox() {
  const controls = document.getElementById('controls');
  const searchContainer = document.createElement('div');
  searchContainer.id = 'search-container';

  const input = document.createElement('input');
  input.type = 'text';
  input.id = 'pac-input';
  input.placeholder = 'Search places...';

  searchContainer.appendChild(input);
  document.body.insertBefore(searchContainer, controls);

  const autocomplete = new google.maps.places.Autocomplete(input);
  autocomplete.bindTo('bounds', map);
  autocomplete.setFields(['geometry', 'name', 'formatted_address', 'place_id']);

  autocomplete.addListener('place_changed', () => {
    const place = autocomplete.getPlace();

    if (!place.geometry) {
      alert('No details available for input: "' + place.name + '"');
      return;
    }

    if (place.geometry.viewport) {
      map.fitBounds(place.geometry.viewport);
    } else {
      map.setCenter(place.geometry.location);
      map.setZoom(17);
    }

    setDestination(place.geometry.location);
    input.value = '';
  });
}
