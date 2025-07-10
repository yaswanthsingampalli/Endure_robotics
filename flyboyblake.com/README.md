# claybeabout.com

### Devops 

- Firebase will reference the `build` directory for deployment
- When updating run `npm run build` to place all new changes into the `build` directory. Followed by `firebase deploy` to submit changes to firebase.
- Dance whenever possible

### Deployment

```
# commit code 
git push
# add changes to build dir
npm run build
# login to firebase
firebase login:ci
# copy output token and run firebase deploy again with --token flag
firebase deploy --token $token
```
