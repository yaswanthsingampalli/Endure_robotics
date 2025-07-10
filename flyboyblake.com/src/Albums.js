import React from 'react';
import './index.scss';
import FadeIn from 'react-fade-in';
import { 
  makeStyles,
  Grid,
  Card,
} from '@mui/material';

const classes = makeStyles((theme) => ({
  root: {
    flexGrow: 1,
  },
  card: {
    backgroundColor: "black",
    padding: 0,
  },
  content: {
    paddingLeft: 0,
  }
}));

export default function Albums() {
  return (
    <FadeIn>
      <h4>work in progress.</h4>
      {/* <Grid container spacing={2}>
      
        <Grid item s={12} md={6}>
          <Card className={classes.card}>
          </Card>
        </Grid>
      </Grid> */}
    </FadeIn>
  );
}