import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';
import Control from './control';
import Info from "./info"
import { BrowserRouter, Switch, Route } from 'react-router-dom'


ReactDOM.render(
    <BrowserRouter>
      <Switch>
        <Route exact path="/" component={Control}/>
        <Route path="/info" component={Info}/>
      </Switch>
    </BrowserRouter>
  ,
  document.getElementById('root')
);


