import React from 'react';
import './index.scss';
import FadeIn from 'react-fade-in';
import { withStyles } from '@mui/material/styles';
import {
  Button,
  Card,
  CardContent,
  makeStyles,
  Grid,
} from '@mui/material';
import {
  reads
} from './Data'

const useStyles = makeStyles((theme) => ({
  root: {
    flexGrow: 1,
  },
  card: {
    backgroundColor: "black",
    color: "white",
    padding: 0,
  },
  content: {
    paddingLeft: 0,
  }
}));

const DateRangeBtn = withStyles({
  contained: {
    marginLeft: '1rem',
    color: 'white',
    fontWeight: 500,
    padding: '0.25rem 0.25rem', 
    backgroundColor: 'black',
    fontFamily: 'Barlow, sans-serif',
  }
})(Button);

export default function Reads() {
  const classes = useStyles();
  return (
    <FadeIn className={classes.root}>
      <Grid container spacing={2}>
        {reads.map(read => (
          <Grid item s={12} md={6}>
            <Card className={classes.card}>
              <CardContent className={classes.content}>
                <h3>
                  <a>{ read.title }</a>
                  <DateRangeBtn variant="contained">{read.author}</DateRangeBtn>
                </h3>
                <p>{ read.desc }</p>
              </CardContent>
            </Card>
          </Grid>
        ))}
      </Grid>
    </FadeIn>
  )
}