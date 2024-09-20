// Auto-generated. Do not edit!

// (in-package line_follower_robot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LineFollowerStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LineFollowerStatus
    // Serialize message field [status]
    bufferOffset = _arraySerializer.int32(obj.status, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LineFollowerStatus
    let len;
    let data = new LineFollowerStatus(null);
    // Deserialize message field [status]
    data.status = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.status.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'line_follower_robot/LineFollowerStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '672ccc7edd3a5621529e608443618e1f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LineFollowerStatus(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = []
    }

    return resolved;
    }
};

module.exports = LineFollowerStatus;
