// Auto-generated. Do not edit!

// (in-package postbot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BoxInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.colors = null;
      this.x = null;
      this.y = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('colors')) {
        this.colors = initObj.colors
      }
      else {
        this.colors = [];
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = [];
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = [];
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoxInfo
    // Serialize message field [colors]
    bufferOffset = _arraySerializer.string(obj.colors, buffer, bufferOffset, null);
    // Serialize message field [x]
    bufferOffset = _arraySerializer.float32(obj.x, buffer, bufferOffset, null);
    // Serialize message field [y]
    bufferOffset = _arraySerializer.float32(obj.y, buffer, bufferOffset, null);
    // Serialize message field [status]
    bufferOffset = _arraySerializer.int32(obj.status, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoxInfo
    let len;
    let data = new BoxInfo(null);
    // Deserialize message field [colors]
    data.colors = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [x]
    data.x = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [y]
    data.y = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [status]
    data.status = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.colors.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 4 * object.x.length;
    length += 4 * object.y.length;
    length += 4 * object.status.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'postbot/BoxInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d695438bc8e2030477534c7599a981b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] colors
    float32[] x
    float32[] y
    int32[] status
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoxInfo(null);
    if (msg.colors !== undefined) {
      resolved.colors = msg.colors;
    }
    else {
      resolved.colors = []
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = []
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = []
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = []
    }

    return resolved;
    }
};

module.exports = BoxInfo;
