import React, { useState, useEffect } from "react";
import "./info.css";
import axios from "axios";
import ScrollBar from 'react-custom-scrollbars'

const Info = () => {
  const [recordList, setRecordList] = useState([]);
  const [batteryCapacity, setBatteryCapacity] = useState(0);

  const getRecordFromBackend = () => {
    // axios to get data from backend
    //TODO
  };

  const renderRecord = () => {
    // render record into web
    let record_list = [];
    //TODO

    //setRecordList(record_list)
  };

  const test_renderRecord = () => {
    let record_list = []
    for (let i = 0 ; i < 10 ; i++ ){
    record_list.push(<p>Room has been disinfected at {Date.now()}</p>)
    record_list.push(<p>Bathroom has been disinfected at {Date.now()}</p>)
    }

    setRecordList(record_list)
  }

  const getBatteryCapacity = () => {
    //TODO
    //return batteryCapacity
    setBatteryCapacity(0.67)
    return 0.67; //test
  };

  const updateBatteryCapacity = (batteryCapacity) => {
    let root = document.documentElement;

    root.style.setProperty("--battery-capacity-size", batteryCapacity);
  };

  useEffect(() => {
    updateBatteryCapacity(getBatteryCapacity());
    test_renderRecord()
  });

  return (
    <div>
      <div>
        <div id="battery" className="d-flex mx-auto">
          <div id="battery-capacity">

          </div>
        </div>
        <div className="d-flex justify-content-center">
            <p id="battery-capacity-text" className="mt-1">
                {batteryCapacity*100+"%"}
            </p>
        </div>
      </div>
      <div id="record" className="col-11 mx-auto mt-1">
          <ScrollBar>
            {recordList}
          </ScrollBar>
      </div>
    </div>
  );
};

export default Info;
