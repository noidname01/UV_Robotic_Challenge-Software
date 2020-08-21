// Auto-generated. Do not edit!

// (in-package practice.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class my_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type my_srvRequest
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type my_srvRequest
    let len;
    let data = new my_srvRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'practice/my_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ef7df1d34137d3879d089ad803388efa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new my_srvRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

class my_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.gender = null;
      this.age = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('gender')) {
        this.gender = initObj.gender
      }
      else {
        this.gender = '';
      }
      if (initObj.hasOwnProperty('age')) {
        this.age = initObj.age
      }
      else {
        this.age = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type my_srvResponse
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [gender]
    bufferOffset = _serializer.string(obj.gender, buffer, bufferOffset);
    // Serialize message field [age]
    bufferOffset = _serializer.int64(obj.age, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type my_srvResponse
    let len;
    let data = new my_srvResponse(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [gender]
    data.gender = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [age]
    data.age = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    length += object.gender.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'practice/my_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76c521bbfe60b764a6e5c48ba35bd8f9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    string gender
    int64 age
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new my_srvResponse(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.gender !== undefined) {
      resolved.gender = msg.gender;
    }
    else {
      resolved.gender = ''
    }

    if (msg.age !== undefined) {
      resolved.age = msg.age;
    }
    else {
      resolved.age = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: my_srvRequest,
  Response: my_srvResponse,
  md5sum() { return 'b8a2aab5099fa54e0eb1247d552c20ed'; },
  datatype() { return 'practice/my_srv'; }
};
