import React, { useState } from "react";
import "./control.css";

import forward from "./image/forward.png";
import backward from "./image/backward.png";
import left from "./image/left.png";
import right from "./image/right.png";
import lighton from "./image/lighton.png";
import lightoff from "./image/lightoff.png";
// import battery from "./image/battery.png";

import camera_stream from "./image/XD_Color.png";
import combined_map from "./image/combine_map.jpg";
// import Axios from "axios";

let ROSLIB = require("roslib");

const Control = (props) => {

  const [lastScale, setLastScale] = useState(null);
  const [isOnNOff, setIsOnNOff] = useState(false); // false means lights off, true means lights on
  const [combinedMap, setCombinedMap] = useState("");
  const [cameraStream, setCameraStream] = useState("");

  let ros = new ROSLIB.Ros({
    url: "ws://192.168.0.201:9090",
  });

  ros.on("connection", () => {
    alert("Connected to websocket server.");
  });

  ros.on("error", (error) => {
    // alert("Error connecting to websocket server: ", error);
  });

  ros.on("close", () => {
    // alert("Connection to websocket server closed.");
  });

  let cmdVel = new ROSLIB.Service({
    ros: ros,
    name: "/cmdToRpiService",
    serviceType: "/uv-robot-ros/cmdToRpi",
  });

  const createRequest = (cmdType, dist_or_deg) => {
    let request = new ROSLIB.ServiceRequest({
      cmdType: cmdType,
      dist_or_deg: dist_or_deg,
    });

    return request;
  };

  const robotAction = (e) => {
    e.preventDefault();
    let action = e.target.name;

    // alert(action);
    /* let request = createRequest("h","0");
    cmdVel.callService(request, (result) => {
      if (!result.isComplete) {
        alert(result.errorMsg)
      } 
    }) */

    if (action === "forward") {
      let request = createRequest("f", "0");
      cmdVel.callService(request, (result) => {
        if (result.isComplete) {
          if (lastScale) {
            lastScale.style = "transform: scale(1)";
          }
          e.target.style = "transform: scale(1.1)";
          setLastScale(e.target);
        } else {
          alert(result.errorMsg);
        }
      });
    } else if (action === "backward") {
      let request = createRequest("b", "0");
      cmdVel.callService(request, (result) => {
        if (result.isComplete) {
          if (lastScale) {
            lastScale.style = "transform: scale(1)";
          }
          e.target.style = "transform: scale(1.1)";
          setLastScale(e.target);
        } else {
          alert(result.errorMsg);
        }
      });
    } else if (action === "left") {
      let request = createRequest("l", "0");
      cmdVel.callService(request, (result) => {
        if (result.isComplete) {
          if (lastScale) {
            lastScale.style = "transform: scale(1)";
          }
          e.target.style = "transform: scale(1.1)";
          setLastScale(e.target);
        } else {
          alert(result.errorMsg);
        }
      });
    } else if (action === "right") {
      let request = createRequest("r", "0");
      cmdVel.callService(request, (result) => {
        if (result.isComplete) {
          if (lastScale) {
            lastScale.style = "transform: scale(1)";
          }
          e.target.style = "transform: scale(1.1)";
          setLastScale(e.target);
        } else {
          alert(result.errorMsg);
        }
      });
    } else if (action === "lighton") {
      let request = createRequest("o", "0");
      cmdVel.callService(request, (result) => {
        if (result.isComplete) {
          setIsOnNOff(true);
        } else {
          alert(result.errorMsg);
        }
      });

      // setIsOnNOff(true)
    } else if (action === "lightoff") {
      let request = createRequest("k", "0");
      cmdVel.callService(request, (result) => {
        if (result.isComplete) {
          setIsOnNOff(false);
        } else {
          alert(result.errorMsg);
        }
      });
      // setIsOnNOff(false)
    } else if (action === "halt") {
      let request = createRequest("h", "0");
      cmdVel.callService(request, (result) => {
        if (result.isComplete) {
          if (lastScale) {
            lastScale.style = "transform: scale(1)";
          }
          e.target.style = "transform: scale(1.1)";
          setLastScale(e.target);
        } else {
          alert(result.errorMsg);
        }
      });
    } else {
      alert("Cannot recognize this command!!");
    }
  };

  // according to the light status to render different appearance
  const onf = () => {
    if (isOnNOff) {
      return (
        <button
          id="lightoff"
          className="light"
          name="lightoff"
          onClick={robotAction}
        >
          <img
            src={lighton}
            className="light-img"
            alt="lighton"
            name="lightoff"
          />
        </button>
      );
    } else {
      return (
        <button
          id="lighton"
          className="light"
          name="lighton"
          onClick={robotAction}
        >
          <img
            src={lightoff}
            className="light-img"
            alt="lightoff"
            name="lighton"
          />
        </button>
      );
    }
  };

  // const updateImg = () => {
  //   Axios.post("http://192.168.0.203/backend/updateImg.php",{}).then(
  //     (data) => {
  //       /* setCombinedMap(data.data.combined_img_path)
  //       setCameraStream(data.data.camera_stream_img_path) */
  //     }
  //   )
  // }

  // setInterval(()=>{
  //   updateImg()
  // },1000)

  return (
    <div id="uvcbot" className="row">
      <div
        id="map-and-camera"
        className="d-block col-12 col-sm-6 col-md-6 col-lg-7"
      >
        <div className="d-flex flex-column justify-content-center col-12 mx-auto">
          {/* <canvas id="combined-map" className="mx-auto"></canvas> */}
          <div id="combined-map" className="mx-auto">
            <img className="img-fluid" src={combined_map} alt="combine_map"/>
          </div>
          <div id="camera-stream" className="mx-auto">
            {/* for Realsense Camera D435 */}
            <img
              className="img-fluid"
              src={camera_stream}
              alt="camera_stream"
            />
          </div>
        </div>
      </div>
      <div
        id="remote-controler"
        className="d-flex align-items-center justify-content-center col-12 col-sm-6 col-md-6 col-lg-5 mt-3 mt-sm-0"
      >
        
        <div id="remote-controler-btn-container" className="m-auto">
          <button id="halt" name="halt" onClick={robotAction}></button>
          {onf()}
          <div
            id="forward"
            className="remote-controler-btns"
            name="forward"
            onClick={robotAction}
          >
            <img
              className="remote-controler-btnimage"
              src={forward}
              alt="forward"
              name="forward"
            />
          </div>
          <div
            id="backward"
            className="remote-controler-btns"
            name="backward"
            onClick={robotAction}
          >
            <img
              className="remote-controler-btnimage"
              src={backward}
              alt="backward"
              name="backward"
            />
          </div>
          <div
            id="left"
            className="remote-controler-btns"
            name="left"
            onClick={robotAction}
          >
            <img
              className="remote-controler-btnimage"
              src={left}
              alt="left"
              name="left"
            />
          </div>
          <div
            id="right"
            className="remote-controler-btns"
            name="right"
            onClick={robotAction}
          >
            <img
              className="remote-controler-btnimage"
              src={right}
              alt="right"
              name="right"
            />
          </div>
        </div>
      </div>
    </div>
  );
};

export default Control;
