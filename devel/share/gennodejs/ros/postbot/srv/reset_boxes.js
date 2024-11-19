// Auto-generated. Do not edit!

// (in-package postbot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class reset_boxesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reset_boxesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reset_boxesRequest
    let len;
    let data = new reset_boxesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'postbot/reset_boxesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new reset_boxesRequest(null);
    return resolved;
    }
};

class reset_boxesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.done = null;
    }
    else {
      if (initObj.hasOwnProperty('done')) {
        this.done = initObj.done
      }
      else {
        this.done = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reset_boxesResponse
    // Serialize message field [done]
    bufferOffset = _serializer.bool(obj.done, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reset_boxesResponse
    let len;
    let data = new reset_boxesResponse(null);
    // Deserialize message field [done]
    data.done = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'postbot/reset_boxesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89bb254424e4cffedbf494e7b0ddbfea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool done
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new reset_boxesResponse(null);
    if (msg.done !== undefined) {
      resolved.done = msg.done;
    }
    else {
      resolved.done = false
    }

    return resolved;
    }
};

module.exports = {
  Request: reset_boxesRequest,
  Response: reset_boxesResponse,
  md5sum() { return '89bb254424e4cffedbf494e7b0ddbfea'; },
  datatype() { return 'postbot/reset_boxes'; }
};
