import React from 'react';
import './index.scss';
import FadeIn from 'react-fade-in';
import claysLetterboxReviews from "./Reviews.json"
import themes from "./themes.json"
import { FaStar, FaStarHalfAlt, FaRegCommentAlt } from 'react-icons/fa';

function compareWatchedDate(a, b) {
  return a["Watched Date"] < b["Watched Date"];
}

function compareName(a, b) {
  return a["Name"] > b["Name"];
}

function compareYear(a, b) {
  return a["Year"] < b["Year"];
}

function compareRating(a, b) {
  return a["Rating"] < b["Rating"];
}

class Letterbox extends React.Component {
  constructor(props) {
    super(props)
    this.state = {
      clickedReview: null,
      reviews: claysLetterboxReviews.reviews.sort(compareWatchedDate),
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
    this.revealPoster = this.revealPoster.bind(this);
  }

  starsHTML(r) {
    // some ratings are 2.5 or 3.5. It's easier to deal with whole numbers. So ex. 3.5 -> 7
    let rating = r * 2
    const stars = []
    while (rating > 0) {
      if (rating >= 2) {
        stars.push(<FaStar style={{ color: themes[this.state.theme]["primary"] }} />);
        rating = rating - 2;
      } else if (rating === 1) {
        stars.push(<FaStarHalfAlt style={{ color: themes[this.state.theme]["primary"] }} />);
        rating = rating - 1;
      } else { }
    }
    return stars;
  }

  convertStringToHTML(htmlString) {
    const parser = new DOMParser();
    const html = parser.parseFromString(htmlString, 'text/html');
    console.log('html');
    console.log(html);
    return html;
  }

  handleChange(e) {
    console.log(e.target.value);
    this.state.reviews = claysLetterboxReviews.reviews.sort(compareWatchedDate);
  }

  handleClick(e) {
    let reviews = document.getElementById("reviews");
    let detailedReview = document.getElementById("detailedReview");
    let reviewText = null;
    let detailedReviewText = null;

    if (this.state.clickedReview != null) {
      this.state.clickedReview = e.currentTarget;
    }
    // if the review's x was clicked
    if (e.currentTarget.className === "close") {
      detailedReview.style.display = "none";
      reviews.style.display = "flex";
    } else {
      for ( let i = 0; i < e.currentTarget.children.length; i++ ) {
        if (e.currentTarget.children[i].className === 'reviewText') {
          console.log("found <div class=\"reviewText\"")
          reviewText = e.currentTarget.children[i];
          break;
        }
      }
      if(reviewText) {
        for ( let i = 0; i < detailedReview.children.length; i++ ) {
          if (detailedReview.children[i].className === "reviewText" ) {
            detailedReviewText = detailedReview.children[i];
            detailedReviewText.innerHTML = reviewText.innerHTML;

            reviews.style.display = "none";
            detailedReview.style.display = "block";
            break;
          }
        }
      }
    }
  }

  revealPoster(e) {
    let review = e.currentTarget;
    review.style.opacity = 1;
  }

  render() {
    return(
      <FadeIn>
        <h2 style={{ marginTop: '3rem', color: themes[this.state.theme]["alternative"] }}>reviews.</h2>
        <hint style={{ color: themes[this.state.theme]["primary"] }}>
          imported from letterboxd
        </hint>
          <p style={{ padding: '1rem', color: themes[this.state.theme]["primary"] }}>
            Sort by
          <select style={{ color: themes[this.state.theme]["primary"], textDecorationColor: themes[this.state.theme]["alternative"] }} defaultValue="watched_date" onChange={(e) => {
              console.log(e.target.value);
              if(e.target.value === "Name") {
                console.log("in Name");
                this.setState({ reviews: claysLetterboxReviews.reviews.sort(compareName) });
              } else if (e.target.value === "Year") {
                console.log("in Year");
                this.setState({ reviews: claysLetterboxReviews.reviews.sort(compareYear) });
              } else if (e.target.value === "Rating") {
                console.log("in Rating");
                this.setState({ reviews: claysLetterboxReviews.reviews.sort(compareRating) });
              } else {
                console.log("in Date");
                this.setState({ reviews: claysLetterboxReviews.reviews.sort(compareWatchedDate) });
              }
            }}>
              <option value="Date">Watched Date</option>
              <option value="Name">Name</option>
              <option value="Year">Year</option>
              <option value="Rating">Rating</option>
            </select>
          </p>
        <div id="detailedReview" style={{ border: `2px ${themes[this.state.theme]["alternative"]} solid` }} className="review-detailed">
          <span className="close" style={{ color: themes[this.state.theme]["primary"] }} onClick={(e) => {
            this.handleClick(e)
          }}>&times;</span>
          <br />
          <br />
          <div className="reviewText" style={{ color: themes[this.state.theme]["primary"] }} />
        </div>
        <div id="reviews" className="reviews">
          { this.state.reviews.map(review => (
            <div 
              className='review' 
              style={{ borderColor: themes[this.state.theme]["alternative"] }} 
              onClick={(e) => {
                this.handleClick(e);
              }}
            >
              <div className='poster-wrapper'>
                <img className='poster' style={{ transition: "opacity 2.0s", opacity: 0 }} src={review.PosterURL} onLoad={ (e) => { this.revealPoster(e) } }/>
              </div>
              <br />
              { this.starsHTML(review.Rating) }
              <FaRegCommentAlt style={{ color: themes[this.state.theme]["primary"] }} className='commentIcon' />
              <div className='reviewText' dangerouslySetInnerHTML={{ __html: review.Review.replace("\n", "<br/><br/>") }}></div>
            </div>
          ))}
        </div>
      </FadeIn>
    );
  }
}

export default Letterbox;
