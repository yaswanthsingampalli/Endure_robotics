import React from 'react';
import './index.scss';
import { FaGithub, FaGoogle, FaLinkedin } from 'react-icons/fa';
import FadeIn from 'react-fade-in';

class Bio extends React.Component {
  render() {
    return (
      <FadeIn>
        <div>
          <a className="fa-link" href='https://github.com/beabout'>
            <FaGithub></FaGithub>
          </a>
          <a className="fa-link" href='mailto:beaboutclayton@gmail.com'>
            <FaGoogle></FaGoogle>
          </a>
          <a className="fa-link" href="https://www.linkedin.com/in/clayton-beabout/">
            <FaLinkedin></FaLinkedin>
          </a>
        </div>
      </FadeIn>
    )
  }
}

export default Bio;