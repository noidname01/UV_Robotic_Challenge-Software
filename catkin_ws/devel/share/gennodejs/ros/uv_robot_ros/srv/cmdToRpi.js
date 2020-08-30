// Auto-generated. Do not edit!

// (in-package uv_robot_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class cmdToRpiRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmdType = null;
      this.dist_or_deg = null;
    }
    else {
      if (initObj.hasOwnProperty('cmdType')) {
        this.cmdType = initObj.cmdType
      }
      else {
        this.cmdType = '';
      }
      if (initObj.hasOwnProperty('dist_or_deg')) {
        this.dist_or_deg = initObj.dist_or_deg
      }
      else {
        this.dist_or_deg = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmdToRpiRequest
    // Serialize message field [cmdType]
    bufferOffset = _serializer.string(obj.cmdType, buffer, bufferOffset);
    // Serialize message field [dist_or_deg]
    bufferOffset = _serializer.string(obj.dist_or_deg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmdToRpiRequest
    let len;
    let data = new cmdToRpiRequest(null);
    // Deserialize message field [cmdType]
    data.cmdType = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [dist_or_deg]
    data.dist_or_deg = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.cmdType.length;
    length += object.dist_or_deg.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uv_robot_ros/cmdToRpiRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f1394a6d86922d90d018be9bd88aab6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string cmdType
    string dist_or_deg
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmdToRpiRequest(null);
    if (msg.cmdType !== undefined) {
      resolved.cmdType = msg.cmdType;
    }
    else {
      resolved.cmdType = ''
    }

    if (msg.dist_or_deg !== undefined) {
      resolved.dist_or_deg = msg.dist_or_deg;
    }
    else {
      resolved.dist_or_deg = ''
    }

    return resolved;
    }
};

class cmdToRpiResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isComplete = null;
      this.errorMsg = null;
    }
    else {
      if (initObj.hasOwnProperty('isComplete')) {
        this.isComplete = initObj.isComplete
      }
      else {
        this.isComplete = false;
      }
      if (initObj.hasOwnProperty('errorMsg')) {
        this.errorMsg = initObj.errorMsg
      }
      else {
        this.errorMsg = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmdToRpiResponse
    // Serialize message field [isComplete]
    bufferOffset = _serializer.bool(obj.isComplete, buffer, bufferOffset);
    // Serialize message field [errorMsg]
    bufferOffset = _serializer.string(obj.errorMsg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmdToRpiResponse
    let len;
    let data = new cmdToRpiResponse(null);
    // Deserialize message field [isComplete]
    data.isComplete = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [errorMsg]
    data.errorMsg = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.errorMsg.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uv_robot_ros/cmdToRpiResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6ac61039326de273907480d4c529a6e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isComplete
    string errorMsg
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmdToRpiResponse(null);
    if (msg.isComplete !== undefined) {
      resolved.isComplete = msg.isComplete;
    }
    else {
      resolved.isComplete = false
    }

    if (msg.errorMsg !== undefined) {
      resolved.errorMsg = msg.errorMsg;
    }
    else {
      resolved.errorMsg = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: cmdToRpiRequest,
  Response: cmdToRpiResponse,
  md5sum() { return 'ab56ca80f4b06b7d4d8f78fb733348e9'; },
  datatype() { return 'uv_robot_ros/cmdToRpi'; }
};
