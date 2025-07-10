import React from "react";
import "./index.scss";
import { Grid } from '@mui/material';
import FadeIn from 'react-fade-in';
import $ from "jquery";
import claySpotifyAlbumsJSON from './Spotify.json';
import themes from "./themes.json"

export const authEndpoint = 'https://accounts.spotify.com/authorize';
const redirectUri = "https://claybeabout.com/discography";
const sp_api_url = "https://api.spotify.com/v1"
const get_user_albums = `${sp_api_url}/me/albums?limit=50&`;

const scopes = [
  "user-library-read",
];

// Get the hash of the url
const hash = window.location.hash
  .substring(1)
  .split("&")
  .reduce(function (initial, item) {
    if (item) {
      var parts = item.split("=");
      initial[parts[0]] = decodeURIComponent(parts[1]);
    }
    return initial;
  }, {});

window.location.hash = "";

function getClientID(){
  return "fed1c759d0d140f591fddf0ba2689b1b";
}

class Spotify extends React.Component {
  constructor(props) {
    super(props)
    this.state = { 
      token: null,
      clayAlbums: claySpotifyAlbumsJSON.albums,
      albums: [],
      theme: props.theme
    }

    var btn = document.getElementById("themeBtn")
    if (btn != null) {
      btn.addEventListener("click", () => {
        setTimeout(() => {
          var theme = document.getElementById("currentTheme")
          this.setState({ theme: theme.outerText })
        }, 10);
      });
    }

    this.getUserAlbums = this.getUserAlbums.bind(this);
    this.revealCover = this.revealCover.bind(this);
  }
  

  updateAlbums = (res) => {
    res.items.forEach(elem => {
      this.state.albums.push({
        art: elem.album.images[0].url,
        artists: [
          elem.album.artists.map(artist => { 
            return { 
              name: artist.name, 
              href: artist.href 
            }
          })
        ],
        name: elem.album.name,
        url: elem.album.external_urls.spotify
      })
    });
    this.setState({
      albums: this.state.albums
    });
  }

  getUserAlbums = (token, offset) => {
    $.ajax({
      url: get_user_albums + 'offset=' + offset,
      type: 'GET',
      headers: {
        'Authorization': 'Bearer ' + token
      },
      success: function (res) {
        if(res.items.length > 0){
          this.updateAlbums(res);
          this.getUserAlbums(token, offset+50)
        }
      }.bind(this)
    });
  }

  componentDidMount() {
    let _token = hash.access_token;
    if (_token) {
      this.setState({
        token: _token
      });
      this.getUserAlbums(_token, 0)
    }
  }

  revealCover(e) {
    let review = e.currentTarget;
    review.style.opacity = 1;
  }


  render() {
    return (
      <FadeIn>
        <div>
          <h2 
            style={{ marginTop: '3rem', color: themes[this.state.theme]["alternative"] }}>
            records.
          </h2>
          <hint 
            style={{ color: themes[this.state.theme]["primary"] }}
          >
            imported from spotify
          </hint>
          <Grid 
            className='p-5' 
            container 
            spacing={1}
          >
            {this.state.clayAlbums.map(
                album => (
              <Grid 
                item
                className='album'
                sm={2} 
                md={2}>
                <a 
                  href={album.url} 
                  target="_blank">
                  <img 
                    src={album.cover} 
                    style={{ transition: "opacity 2.0s", opacity: 0 }}
                    onLoad={this.revealCover}
                    className='album-art' />
                </a>
              </Grid>
            ))}
          </Grid>
        </div>
      </FadeIn>
    );
  }
}
export default Spotify;