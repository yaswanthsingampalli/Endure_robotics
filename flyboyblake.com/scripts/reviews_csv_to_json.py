#!/usr/bin/python3

import csv
import re
import subprocess

reviews = 'files/reviews.csv'
ratings = 'files/ratings.csv'
outfile = 'reviews.json'

arr = []

def grab_url_and_review(row):
  if 'Review' in row.keys():
    review = re.sub("\xa0", "", re.sub("\n", "<br/>", re.sub("\"", "\\\"", row['Review'])))
  else:
    review = ""
  subprocess.run("wget %s" % row['Letterboxd URI'], shell=True)
  poster_file = re.sub("https://boxd.it/", "", row['Letterboxd URI'])
  # grab the poster_url from the downloaded review_url
  poster_url = subprocess.run("grep -Eo \"itemReviewed.*jpg\" %s | sed 's/itemReviewed\":{\"image\":\"//' " % poster_file, shell=True, stdout=subprocess.PIPE)
  poster_url = poster_url.stdout.decode('ascii')
  poster_url = re.sub('\n', '', poster_url)
  return poster_url, review

def read_reviews():
  with open(reviews, 'r') as infile:
    reader = csv.DictReader(infile)
    for row in reader:
      url, review = grab_url_and_review(row)
      arr.append({
        'Watched Date': row['Watched Date'],
        'Date': row['Date'],
        'Name': row['Name'],
        'Year': row['Year'],
        'Rating': row['Rating'],
        'Review': review,
        'PosterURL': url
      })

def read_ratings():
  with open(ratings, 'r') as infile:
    reader = csv.DictReader(infile)
    for row in reader:
      url, review = grab_url_and_review(row)
      arr.append({
        'Watched Date': row['Date'],
        'Date': row['Date'],
        'Name': row['Name'],
        'Year': row['Year'],
        'Rating': row['Rating'],
        'Review': review,
        'PosterURL': url
      })

def write_out(sorted_arr):
    of = open(outfile, 'w')
    of.write("{ \"reviews\": [\n")
    first = True
    for row in sorted_arr:
      if first:
        first = False
      else:
        of.write(",\n")
      write_line(of, row)
    of.write("\n] }")
      
def write_line(outfile, row):
  outfile.write("{")
  outfile.write("\n\t\"Date\": \"%s\", "   % row['Date'])
  outfile.write("\n\t\"Watched Date\": \"%s\", "   % row['Watched Date'])
  outfile.write("\n\t\"Name\": \"%s\", "   % row['Name'])
  outfile.write("\n\t\"Year\": \"%s\", "   % row['Year'])
  outfile.write("\n\t\"Rating\": \"%s\", " % row['Rating'])
  outfile.write("\n\t\"Review\": \"%s\", " % row['Review'])
  outfile.write("\n\t\"PosterURL\": \"%s\"" % row['PosterURL'])
  outfile.write("\n}")

def generate_json():
  read_reviews()
  read_ratings()
  sorted_arr = sorted(arr, key=lambda d: d['Watched Date'])
  write_out(sorted_arr)


generate_json()
subprocess.run("rm 2*; rm 3*; rm 4*;", shell=True)
