#!/usr/local/bin/python3
# install: `pip3 install spotipy`

import spotipy
import json
import os 
from spotipy.oauth2 import SpotifyOAuth


CLIENT_ID=""
APP_CLIENT_SECRET=""
APP_REDIRECT_URI="http://localhost:3000/discography"

sp = spotipy.Spotify(auth_manager=SpotifyOAuth(client_id=CLIENT_ID,
                                               client_secret=APP_CLIENT_SECRET,
                                               redirect_uri=APP_REDIRECT_URI,
                                               scope="user-library-read"))

albums = []
offset = 0
ting = True

while True:
  result = sp.current_user_saved_albums(limit=50, offset=offset)
  for album in map(lambda x: x['album'], result['items']):
    tracks = []
    for track in album['tracks']['items']:
      if ting:
        ting = False
        print(track)
      t = {
        'name': track['name'],
        'id': track['id'],
        'track_number': track['track_number'],
        'uri': track['uri'],
        'duration_ms': track['duration_ms'],
        'artists': list(map(lambda artist: { 'name': artist['name'], 'id': artist['id'] }, track['artists'])),
      }
      tracks.append(t)

    entry = {
      'id': album['id'],
      'popularity': album['popularity'],
      'name': album['name'],
      'artists': list(map(lambda artist: { 'name': artist['name'], 'id': artist['id'] }, album['artists'])),
      'cover': album['images'][0]['url'],
      'total_tracks': album['total_tracks'],
      'url': album['external_urls']['spotify'],
      'release_date': album['release_date'],
      'tracks': tracks
    }
    albums.append(entry)
  offset += 50

  if len(result['items']) < 50:
    break

 
# Writing to sample.json
with open("Spotify.json", "w") as of:
    of.write("{")
    of.write("\"albums\": [")
    of.write(', '.join(list(map(lambda album: json.dumps(album, indent=4), albums))))
    of.write("\n]")
    of.write("\n}")

print(f'{len(albums)} albums found.')

os.system("cp Spotify.json /Users/clay/Desktop/develop/flyboyblake.com/src/")
