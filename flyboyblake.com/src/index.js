import React from 'react';
import './index.scss';
import ReactDOM from 'react-dom';
import FadeIn from 'react-fade-in';
import themes from './themes.json'

/* Components */
import Catalog from './Catalog'
import Spotify from './Spotify'
import Notes from './Notes'
import Letterbox from './Letterbox'

/* fa icons */
import { 
  FaGithub, 
  FaSpotify, 
  FaFolderOpen, 
  FaAngleDoubleLeft,
  FaFilm,
  FaRegLemon,
  FaRegKissWinkHeart,
  FaLinkedin,
  FaFileDownload
} from 'react-icons/fa';

import {
  BrowserRouter,
  Routes,
  Route,
  Link
} from "react-router-dom";


function atHomepage() {
  if (window.location.pathname === "/") {
    return true
  } else {
    return false
  }
}

Array.prototype.sample = function () {
  return this[Math.floor(Math.random() * this.length)];
}

class App extends React.Component { 
  constructor(props) {
    super(props)
    this.state = { 
      atHome: atHomepage(),
      theme: "dannyphantom"
    }

    this.randomizeTheme = this.randomizeTheme.bind(this);
  }

  randomizeTheme() {
    var theme = this.state.theme
    while (theme == this.state.theme) { theme = Object.keys(themes).sample() }
    this.setState({ theme: theme })
    document.body.style.backgroundColor = themes[theme]["secondary"]
    return theme
  }

  render() {
    return (
      <BrowserRouter>
        <p style={{ display: "none" }} id="currentTheme">{this.state.theme}</p>
        <FaRegLemon id="themeBtn" className="theme-icon" style={{ color: themes[this.state.theme]["primary"] }} onClick={this.randomizeTheme} />
        { atHomepage() ?
          <FadeIn>
            <div className="icons">
              <a 
                className="fa-link" 
                style={{ color: themes[this.state.theme]["primary"] }} 
                href="https://github.com/beabout" 
                target="_blank"
              >
                <FaGithub />
              </a>
              <a 
                className="fa-link" 
                style={{ color: themes[this.state.theme]["primary"] }} 
                href="https://www.linkedin.com/in/clayton-beabout/" 
                target="_blank"
              >
                <FaLinkedin />
              </a>
              <Link
                className="fa-link"
                style={{ color: themes[this.state.theme]["primary"] }}
                to="/discography"
                onClick={() => this.setState({ atHome: true })}
              >
                <FaSpotify />
              </Link>
              <Link
                className="fa-link"
                style={{ color: themes[this.state.theme]["primary"] }}
                to="/films"
                onClick={() => this.setState({ atHome: true })}
              >
                <FaFilm />
              </Link>
              <a 
                className="fa-link" 
                style={{ color: themes[this.state.theme]["primary"] }} 
                href="https://www.laurenbeabout.com/" 
                target="_blank"
              >
                <FaRegKissWinkHeart />
              </a>
              <a
                className="fa-link"
                style={{ color: themes[this.state.theme]["primary"] }}
                href="./claybeabout.pdf"
                target="_blank"
              >
                <FaFileDownload />
              </a>
            </div>
          </FadeIn>
          :
          <Link 
            className="fa-link"
            style={{ color: themes[this.state.theme]["primary"] }}
            to="/" 
            onClick={() => this.setState({ atHome: true })}
          >
            <FaAngleDoubleLeft />
          </Link>
        }
        <div className="revision" style={{ color: themes[this.state.theme]["primary"]}}>flyboyblake.com</div>
        <Routes>
          <Route path="/"></Route>
          <Route path="/catalog" element={<Catalog />} />
          <Route path="/discography" element={<Spotify theme={this.state.theme} />} />
          <Route path="/films" element={<Letterbox theme={this.state.theme} />} />
          <Route path="/pdf" element={<Letterbox theme={this.state.theme} />} />
        </Routes>
      </BrowserRouter>
    );
  };
}

// ========================================

ReactDOM.render(
  <App />,
  document.getElementById('root')
);
