import React, { useState } from 'react';
import './Test.css';


const Test = (props) => {
  
  

  const drawOdom = (mapWidth,mapHeight,dataFromBackEnd) => {

  }

  /* const getMap = () => {
    axios.post("",{})
    .then(
      (data) => {
        console.log(data)
        if(data.data.success === 1){
          
          drawOdom(data.data.mapWidth, data.data.mapHeight, data.data.map)

        } 
        else{
          alert("Something's Wrong!\nCan't get Map!")
        }
      }
    )
  }
 */
  return(
    <div className="row">
      <div id="map-and-camera" className="d-block col-12 col-sm-6 col-md-7 col-lg-8">
        <div className="d-flex flex-column justify-content-center col-12 mx-auto" >
          {/* <canvas id="combined-map" className="mx-auto"></canvas> */}
          <div id="combined-map" className="mx-auto"></div>
          <div id="camera-stream" className="mx-auto">
            {/* for Realsense Camera D435 */}
          </div>
        </div>
      </div>
      <div  className="col-12 col-sm-6 col-md-5 col-lg-4">
        <div id="remote-controler">

        </div>
      </div>
    </div>

  )
}

export default Test;
